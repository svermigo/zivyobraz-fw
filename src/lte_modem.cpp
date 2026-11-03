#include "lte_modem.h"

#ifdef USE_LTE_MODEM

  #include "board.h"
  #include "logger.h"

  #include <HardwareSerial.h>
  #include <Preferences.h>
  #include <driver/gpio.h>
  #include <driver/rtc_io.h>

extern const char *const NVS_NAMESPACE;

// Survives deep sleep, so we know whether the module is still running (and at
// which level EN was latched at) before we release the pad hold.
RTC_DATA_ATTR bool rtc_modemPowered = false;

namespace LteModem
{

namespace
{

HardwareSerial modem(LTE_UART_NUM);

bool g_poweredOn = false;
bool g_ready = false;

int8_t g_rssi = 0;
String g_operator;
String g_imei;
String g_ipAddress;
String g_apn;

// Socket (id 0) state, updated both by explicit commands and by URCs
bool g_sockOpen = false;
bool g_sockRemoteClosed = false;
bool g_sockDataPending = false;
int g_sockPendingRest = 0;

// Set while waiting for the asynchronous +CIPOPEN result
bool g_cipopenSeen = false;
int g_cipopenResult = -1;

// AT+CIPSEND answers "OK" as soon as it has swallowed the payload and only
// later confirms the actual transmission with "+CIPSEND: <id>,<req>,<cnf>".
bool g_cipsendSeen = false;
int g_cipsendConfirmed = 0;

// Same for AT+NETOPEN: "OK" first, then "+NETOPEN: <err>" with 0 meaning
// success. Careful - the query form AT+NETOPEN? reports STATE under the very
// same prefix, where 1 means open, so the two must never be mixed up.
bool g_netopenSeen = false;
int g_netopenResult = -1;

// Restarts the module reported about itself since bring-up began. Counted from
// its own boot URC rather than inferred from a failed baud switch - the module
// can reboot without us noticing otherwise, and a module that keeps rebooting
// never finishes a band scan, which looks exactly like a missing antenna.
// After a cold power-up the first boot URC is expected and not counted.
uint8_t g_moduleRestarts = 0;
bool g_expectBootUrc = false;

constexpr uint32_t UART_RX_BUFFER = 8192;

// Start of the current begin() call, for the phase timing breakdown below.
uint32_t g_beginStart = 0;

///////////////////////////////////////////////
// Low level UART / AT plumbing
///////////////////////////////////////////////

String csvField(const String &value, int index); // defined further down
String extractValue(const String &response, const char *prefix); // defined further down
void readSignalQuality();                                        // defined further down

// Marks how long bring-up has taken so far. A slow wake-up is almost always one
// specific phase, and without this it is guesswork which one.
void phase(const String &name)
{
  Logger::log<Logger::Topic::LTE>("  [{} ms] {}\n", millis() - g_beginStart, name);
}

void drainInput()
{
  while (modem.available())
    modem.read();
}

void writeCommand(const char *cmd)
{
  Logger::log<Logger::Level::DEBUG, Logger::Topic::LTE>(">> {}\n", cmd);
  modem.print(cmd);
  modem.print("\r\n");
}

// Recognise and consume unsolicited result codes. Returns true if the line was
// a URC and should not be treated as a command response.
bool handleUrc(const String &line)
{
  if (line.startsWith("+CIPRXGET: 1"))
  {
    g_sockDataPending = true;
    return true;
  }
  if (line.startsWith("+IPCLOSE:") || line == "CLOSED")
  {
    Logger::log<Logger::Level::DEBUG, Logger::Topic::LTE>("Socket closed by peer ({})\n", line);
    g_sockRemoteClosed = true;
    return true;
  }
  if (line.startsWith("+NETOPEN:"))
  {
    g_netopenResult = line.substring(line.indexOf(": ") + 2).toInt();
    g_netopenSeen = true;
    return true;
  }
  if (line.startsWith("+CIPCLOSE:") || line.startsWith("+NETCLOSE:"))
  {
    // Async confirmation of a close we already issued; nothing to decide.
    return true;
  }
  if (line.startsWith("+CIPSEND:"))
  {
    // +CIPSEND: <id>,<requested>,<confirmed>
    g_cipsendConfirmed = csvField(line.substring(line.indexOf(": ") + 2), 2).toInt();
    g_cipsendSeen = true;
    return true;
  }
  if (line.startsWith("+CIPOPEN:"))
  {
    int comma = line.indexOf(',');
    g_cipopenResult = (comma > 0) ? line.substring(comma + 1).toInt() : -1;
    g_cipopenSeen = true;
    return true;
  }
  if (line.startsWith("+CIPEVENT:"))
  {
    Logger::log<Logger::Level::WARNING, Logger::Topic::LTE>("Network event: {}\n", line);
    g_sockRemoteClosed = true;
    g_sockOpen = false;
    return true;
  }
  if (line == "RDY" || line.startsWith("*ATREADY"))
  {
    if (g_expectBootUrc)
    {
      g_expectBootUrc = false; // the power-up boot we just caused
      Logger::log<Logger::Level::DEBUG, Logger::Topic::LTE>("Module booted\n");
      return true;
    }
    if (g_moduleRestarts < 255)
      g_moduleRestarts++;
    Logger::log<Logger::Level::WARNING, Logger::Topic::LTE>("Module restarted unexpectedly ({} this bring-up)\n",
                                                            g_moduleRestarts);
    return true;
  }

  // Boot chatter and misc notifications we never act on.
  // Note: "+CPIN:" must NOT be listed here - it is also the reply to AT+CPIN?,
  // and swallowing it as a URC would hide the SIM state from waitForSim().
  if (line == "SMS DONE" || line == "PB DONE" || line.startsWith("+CGEV:") || line.startsWith("+SIMCARD:"))
  {
    return true;
  }
  return false;
}

// Reads one CRLF terminated line. Returns false on timeout (out holds whatever
// partial data arrived).
bool readLineRaw(String &out, uint32_t timeoutMs)
{
  out = "";
  uint32_t deadline = millis() + timeoutMs;
  while ((int32_t)(millis() - deadline) < 0)
  {
    if (!modem.available())
    {
      delay(1);
      continue;
    }
    char c = (char)modem.read();
    if (c == '\n')
    {
      if (out.endsWith("\r"))
        out.remove(out.length() - 1);
      return true;
    }
    out += c;
  }
  return false;
}

bool isFinalOk(const String &line) { return line == "OK" || line == "SHUT OK"; }

bool isFinalError(const String &line)
{
  return line == "ERROR" || line.startsWith("+CME ERROR") || line.startsWith("+CMS ERROR") ||
         line.startsWith("+IP ERROR") || line == "NO CARRIER";
}

// Collects intermediate lines into `response` (newline separated) until a final
// result code arrives.
AtResult waitResponse(uint32_t timeoutMs, String *response)
{
  if (response)
    *response = "";

  uint32_t deadline = millis() + timeoutMs;
  String line;

  while ((int32_t)(millis() - deadline) < 0)
  {
    uint32_t remaining = (uint32_t)(deadline - millis());
    if (!readLineRaw(line, remaining))
      break;

    if (line.length() == 0)
      continue;

    Logger::log<Logger::Level::DEBUG, Logger::Topic::LTE>("<< {}\n", line);

    if (handleUrc(line))
      continue;

    if (isFinalOk(line))
      return AtResult::Ok;

    if (isFinalError(line))
    {
      if (response)
        *response = line;
      return AtResult::Error;
    }

    if (response)
    {
      if (response->length() > 0)
        *response += '\n';
      *response += line;
    }
  }

  return AtResult::Timeout;
}

// Waits for the bare '>' data prompt emitted by AT+CIPSEND.
bool waitForPrompt(uint32_t timeoutMs)
{
  uint32_t deadline = millis() + timeoutMs;
  while ((int32_t)(millis() - deadline) < 0)
  {
    if (!modem.available())
    {
      delay(1);
      continue;
    }
    if ((char)modem.read() == '>')
      return true;
  }
  return false;
}

// Reads exactly `len` raw bytes (binary safe, no line parsing).
size_t readRaw(uint8_t *buf, size_t len, uint32_t timeoutMs)
{
  size_t got = 0;
  uint32_t lastData = millis();
  while (got < len)
  {
    int avail = modem.available();
    if (avail > 0)
    {
      int n = modem.readBytes(buf + got, min((size_t)avail, len - got));
      if (n > 0)
      {
        got += n;
        lastData = millis();
        continue;
      }
    }
    if (millis() - lastData > timeoutMs)
      break;
    delay(1);
  }
  return got;
}

// Reads and dispatches whatever arrives within the given window. Used after
// commands whose confirmation URC follows the "OK" by some milliseconds.
void drainUrcs(uint32_t windowMs)
{
  uint32_t deadline = millis() + windowMs;
  while ((int32_t)(millis() - deadline) < 0)
  {
    String line;
    if (!readLineRaw(line, 200) || line.length() == 0)
      continue;
    Logger::log<Logger::Level::DEBUG, Logger::Topic::LTE>("<< {}\n", line);
    handleUrc(line);
  }
}

///////////////////////////////////////////////
// Helpers
///////////////////////////////////////////////

// Extracts the value that follows "<prefix>" in a (possibly multi-line)
// response, trimmed of quotes and whitespace.
String extractValue(const String &response, const char *prefix)
{
  int idx = response.indexOf(prefix);
  if (idx < 0)
    return String();
  int start = idx + strlen(prefix);
  int end = response.indexOf('\n', start);
  String value = (end < 0) ? response.substring(start) : response.substring(start, end);
  value.trim();
  return value;
}

// Returns field `index` (0-based) of a comma separated list.
String csvField(const String &value, int index)
{
  int start = 0;
  for (int i = 0; i < index; i++)
  {
    start = value.indexOf(',', start);
    if (start < 0)
      return String();
    start++;
  }
  int end = value.indexOf(',', start);
  String field = (end < 0) ? value.substring(start) : value.substring(start, end);
  field.trim();
  field.replace("\"", "");
  return field;
}

// Last network we successfully registered on. Narrowing the search to this
// means far fewer transmit bursts during attach, which matters as much for a
// marginal supply as it does for speed.
struct NetworkCache
{
  String oper; // numeric MCC+MNC, e.g. "23001"
  int act = 0; // access technology from +COPS?; 7 = E-UTRAN (LTE)
  int band = 0;

  bool isValid() const { return oper.length() > 0; }
};

NetworkCache loadNetworkCache()
{
  NetworkCache cache;
  Preferences prefs;
  prefs.begin(NVS_NAMESPACE, true);
  cache.oper = prefs.getString("oper", "");
  cache.act = prefs.getInt("act", 0);
  cache.band = prefs.getInt("band", 0);
  prefs.end();
  return cache;
}

void storeNetworkCache(const NetworkCache &cache)
{
  Preferences prefs;
  prefs.begin(NVS_NAMESPACE);
  prefs.putString("oper", cache.oper);
  prefs.putInt("act", cache.act);
  prefs.putInt("band", cache.band);
  prefs.end();
}

void clearNetworkCache()
{
  Preferences prefs;
  prefs.begin(NVS_NAMESPACE);
  prefs.remove("oper");
  prefs.remove("act");
  prefs.remove("band");
  prefs.end();
}

// +CNBP takes 64-bit hex masks where bit n-1 enables LTE band n.
String lteBandMask(int band)
{
  if (band < 1 || band > 64)
    return String();

  uint64_t mask = 1ULL << (band - 1);
  char buf[24];
  snprintf(buf, sizeof(buf), "0X%016llX", (unsigned long long)mask);
  return String(buf);
}

String loadStoredApn()
{
  Preferences prefs;
  prefs.begin(NVS_NAMESPACE, true);
  String apn = prefs.getString("apn", "");
  prefs.end();
  return apn;
}

///////////////////////////////////////////////
// Bring-up steps
///////////////////////////////////////////////

void applyPower(bool on)
{
  pinMode(LTE_PIN_PWR_EN, OUTPUT);
  digitalWrite(LTE_PIN_PWR_EN, on ? HIGH : LOW);
  g_poweredOn = on;
  rtc_modemPowered = on;
}

// Hammer "AT" until the module answers.
bool syncAt(uint32_t timeoutMs)
{
  uint32_t deadline = millis() + timeoutMs;
  while ((int32_t)(millis() - deadline) < 0)
  {
    // Dispatch what is buffered rather than discarding it: a *ATREADY in here
    // is the module (re)booting, and the restart count must not miss it.
    String pending;
    while (modem.available() && readLineRaw(pending, 20))
      if (pending.length() > 0)
        handleUrc(pending);

    modem.print("AT\r\n");
    String resp;
    if (waitResponse(600, &resp) == AtResult::Ok)
      return true;
    delay(200);
  }
  return false;
}

// Raise the UART speed once the module is talking.
void switchBaud()
{
  if (LTE_BAUD == LTE_BOOT_BAUD)
    return;

  char cmd[32];
  snprintf(cmd, sizeof(cmd), "AT+IPR=%d", (int)LTE_BAUD);
  writeCommand(cmd);
  waitResponse(2000, nullptr);

  delay(100);
  modem.updateBaudRate(LTE_BAUD);
  delay(100);

  if (syncAt(3000))
  {
    Logger::log<Logger::Topic::LTE>("UART switched to {} baud\n", (int)LTE_BAUD);
    return;
  }

  Logger::log<Logger::Level::WARNING, Logger::Topic::LTE>("Baud switch to {} failed, staying at {}\n", (int)LTE_BAUD,
                                                          (int)LTE_BOOT_BAUD);
  modem.updateBaudRate(LTE_BOOT_BAUD);
  syncAt(3000);
}

// The module answers AT as soon as its command parser is up, but keeps emitting
// boot notifications (+CPIN, SMS DONE, *ATREADY) for several seconds afterwards
// and drops commands sent in that window. Wait for the line to fall quiet first;
// without this the bring-up races the tail of the module's own boot, and whether
// it wins depends on the exact module.
void waitForBootQuiet(uint32_t quietMs, uint32_t timeoutMs)
{
  uint32_t deadline = millis() + timeoutMs;
  uint32_t lastActivity = millis();

  while ((int32_t)(millis() - deadline) < 0)
  {
    String line;
    if (readLineRaw(line, 200) && line.length() > 0)
    {
      Logger::log<Logger::Level::DEBUG, Logger::Topic::LTE>("<< {}\n", line);
      handleUrc(line);
      lastActivity = millis();
      continue;
    }

    if (millis() - lastActivity >= quietMs)
      return;
  }

  Logger::log<Logger::Level::WARNING, Logger::Topic::LTE>("Module still chatting after {} ms, proceeding anyway\n",
                                                          timeoutMs);
}

// A module reset drops AT+IPR back to the boot baud, so our UART ends up
// talking at the wrong speed and every command reads as dead air. Re-sync and
// re-apply the fast baud instead of burning the whole wait on silence.
bool recoverUart(bool *wasReset = nullptr)
{
  if (wasReset)
    *wasReset = false;

  // A plain AT at the current baud answers whenever the module is merely busy
  // with its cell search rather than actually gone.
  if (syncAt(1500))
    return true;

  Logger::log<Logger::Level::WARNING, Logger::Topic::LTE>("UART silent, re-syncing at {} baud\n",
                                                          (int)LTE_BOOT_BAUD);
  modem.updateBaudRate(LTE_BOOT_BAUD);
  delay(50);

  // A brown-out means the module is rebooting, and that takes a few seconds -
  // allow it the same budget as the initial power-on rather than giving up.
  if (!syncAt(LTE_BOOT_TIMEOUT_MS))
  {
    Logger::log<Logger::Level::ERROR, Logger::Topic::LTE>("Modem unreachable at either baud rate\n");
    return false;
  }

  Logger::log<Logger::Topic::LTE>("Modem came back (it reset), reconfiguring\n");
  if (wasReset)
    *wasReset = true;

  writeCommand("ATE0");
  waitResponse(2000, nullptr);
  drainInput();
  switchBaud();
  return true;
}

bool waitForSim(uint32_t timeoutMs)
{
  uint32_t deadline = millis() + timeoutMs;
  while ((int32_t)(millis() - deadline) < 0)
  {
    String resp;
    writeCommand("AT+CPIN?");
    if (waitResponse(3000, &resp) == AtResult::Ok && resp.indexOf("READY") >= 0)
      return true;
    delay(500);
  }
  Logger::log<Logger::Level::ERROR, Logger::Topic::LTE>("SIM not ready (missing card or PIN locked)\n");
  return false;
}

// +CEREG/+CGREG stat 1 (home) or 5 (roaming) means we're on the network.
// *responded reports whether the modem answered at all, so the caller can tell
// a plain "not registered yet" apart from a dead UART.
bool isRegistered(const char *cmd, bool *responded = nullptr)
{
  String resp;
  writeCommand(cmd);
  AtResult result = waitResponse(3000, &resp);

  if (responded)
    *responded = (result != AtResult::Timeout);

  if (result != AtResult::Ok)
    return false;

  int idx = resp.indexOf(": ");
  if (idx < 0)
    return false;

  int stat = csvField(resp.substring(idx + 2), 1).toInt();
  return stat == 1 || stat == 5;
}

bool waitForRegistration(uint32_t timeoutMs)
{
  uint32_t deadline = millis() + timeoutMs;
  uint8_t silentPolls = 0;

  while ((int32_t)(millis() - deadline) < 0)
  {
    bool lteAnswered = false;
    bool gsmAnswered = false;

    if (isRegistered("AT+CEREG?", &lteAnswered) || isRegistered("AT+CGREG?", &gsmAnswered))
      return true;

    if (lteAnswered || gsmAnswered)
    {
      silentPolls = 0;
    }
    else if (++silentPolls >= 3)
    {
      // Three polls in a row with no answer: the module most likely reset and
      // reverted to the boot baud. Try to get the link back, then keep waiting.
      silentPolls = 0;

      bool wasReset = false;
      if (!recoverUart(&wasReset))
        return false;

      // Only pay the backoff when the module really restarted - on a marginal
      // supply that means it browned out and the rail needs time before it
      // transmits again. A module that was just busy searching needs nothing.
      if (wasReset)
      {
        Logger::log<Logger::Topic::LTE>("Backing off {} ms before retrying\n", (int)LTE_RESET_BACKOFF_MS);
        delay(LTE_RESET_BACKOFF_MS);
      }
    }

    delay(1000);
  }

  // Report what the radio actually hears before giving up. Without this a
  // failure looks identical whether the antenna is missing, the cell is out of
  // reach, or the SIM is not allowed on the network.
  readSignalQuality();

  // Full radio state, so a failure says which of the many causes it was.
  String resp;
  for (const char *query : {"AT+CFUN?", "AT+COPS?", "AT+CNMP?", "AT+CNBP?", "AT+CPIN?", "AT+CPSI?"})
  {
    writeCommand(query);
    if (waitResponse(5000, &resp) != AtResult::Ok || resp.length() == 0)
      continue;

    // The module turns echo back on after a restart, so the response may begin
    // with the command itself - report only the line that carries the value.
    int at = resp.indexOf('+');
    Logger::log<Logger::Level::ERROR, Logger::Topic::LTE>("  {}\n", at >= 0 ? resp.substring(at) : resp);
  }

  Logger::log<Logger::Level::ERROR, Logger::Topic::LTE>(
    "Network registration timed out ({} module restarts during bring-up)\n", g_moduleRestarts);
  return false;
}

void setApn(const String &apn)
{
  g_apn = apn;
  String cmd = "AT+CGDCONT=1,\"IP\",\"" + apn + "\"";
  writeCommand(cmd.c_str());
  waitResponse(3000, nullptr);
}

// Ask the network which APN to use (only meaningful once registered).
String queryNetworkApn()
{
  String resp;
  writeCommand("AT+CGNAPN");
  if (waitResponse(5000, &resp) != AtResult::Ok)
    return String();

  String value = extractValue(resp, "+CGNAPN:");
  if (csvField(value, 0).toInt() != 1)
    return String();

  return csvField(value, 1);
}

bool waitForAttach(uint32_t timeoutMs)
{
  uint32_t deadline = millis() + timeoutMs;
  while ((int32_t)(millis() - deadline) < 0)
  {
    String resp;
    writeCommand("AT+CGATT?");
    if (waitResponse(5000, &resp) == AtResult::Ok && extractValue(resp, "+CGATT:").toInt() == 1)
      return true;
    delay(1000);
  }
  Logger::log<Logger::Level::ERROR, Logger::Topic::LTE>("PDP attach timed out\n");
  return false;
}

void readSignalQuality()
{
  String resp;
  writeCommand("AT+CSQ");
  if (waitResponse(3000, &resp) != AtResult::Ok)
    return;

  int rssiRaw = extractValue(resp, "+CSQ:").toInt();
  // 0..31 maps linearly onto -113..-51 dBm; 99 means "not detectable"
  g_rssi = (rssiRaw >= 0 && rssiRaw <= 31) ? (int8_t)(-113 + 2 * rssiRaw) : 0;
  Logger::log<Logger::Topic::LTE>("Signal: {} dBm (CSQ {})\n", g_rssi, rssiRaw);
}

// AT+CGSN answers with the bare 15-digit IMEI. Pick that line out explicitly:
// if the module restarted, echo is back on and the reply also holds the command.
void readImei()
{
  String resp;
  writeCommand("AT+CGSN");
  if (waitResponse(3000, &resp) != AtResult::Ok)
    return;

  int start = 0;
  while (start <= (int)resp.length())
  {
    int end = resp.indexOf('\n', start);
    String line = (end < 0) ? resp.substring(start) : resp.substring(start, end);
    line.trim();

    bool digits = line.length() >= 14;
    for (size_t i = 0; digits && i < line.length(); i++)
      digits = isDigit(line[i]);
    if (digits)
    {
      g_imei = line;
      return;
    }

    if (end < 0)
      break;
    start = end + 1;
  }
}

// The factory band mask and the cached network describe one particular module.
// If a different module is now fitted (it happens - carriers get swapped), they
// are stale: the old mask may not match this variant's bands, and the cached
// band may not even exist for it. Drop both and start this module afresh.
void forgetOtherModule()
{
  if (g_imei.length() == 0)
    return;

  Preferences prefs;
  prefs.begin(NVS_NAMESPACE);
  String known = prefs.getString("imei", "");
  if (known != g_imei)
  {
    if (known.length() > 0)
      Logger::log<Logger::Topic::LTE>("Different modem fitted (IMEI {} -> {}), discarding its cached settings\n", known,
                                      g_imei);
    prefs.remove("cnbp0");
    prefs.remove("oper");
    prefs.remove("act");
    prefs.remove("band");
    prefs.putString("imei", g_imei);
  }
  prefs.end();
}

void readIdentity()
{
  String resp;

  // Ask for the long alphanumeric name, otherwise +COPS? answers "23001"
  writeCommand("AT+COPS=3,0");
  waitResponse(3000, nullptr);

  writeCommand("AT+COPS?");
  if (waitResponse(5000, &resp) == AtResult::Ok)
  {
    // +COPS: <mode>,<format>,"<operator>",<act>
    String value = extractValue(resp, "+COPS:");
    String name = csvField(value, 2);
    if (name.length() > 0)
      g_operator = name;
  }
}

// The module's factory band mask, captured once before we ever narrow it, so
// widening again never has to guess which bands this variant supports.
String loadFullBandScope()
{
  Preferences prefs;
  prefs.begin(NVS_NAMESPACE, true);
  String scope = prefs.getString("cnbp0", "");
  prefs.end();
  return scope;
}

void rememberFullBandScope()
{
  if (loadFullBandScope().length() > 0)
    return;

  String resp;
  writeCommand("AT+CNBP?");
  if (waitResponse(3000, &resp) != AtResult::Ok)
    return;

  String scope = extractValue(resp, "+CNBP:");
  if (scope.length() == 0)
    return;

  Preferences prefs;
  prefs.begin(NVS_NAMESPACE);
  prefs.putString("cnbp0", scope);
  prefs.end();
  Logger::log<Logger::Level::DEBUG, Logger::Topic::LTE>("Stored factory band scope: {}\n", scope);
}

// Only write +CNMP / +CNBP when the value actually changes - both live in the
// module's NVM and there is no reason to wear it out every wake-up.
void setIfDifferent(const char *query, const char *prefix, const char *setCmd, const String &wanted)
{
  String resp;
  writeCommand(query);
  if (waitResponse(3000, &resp) == AtResult::Ok && extractValue(resp, prefix) == wanted)
    return;

  String cmd = String(setCmd) + wanted;
  writeCommand(cmd.c_str());
  waitResponse(5000, nullptr);
}

// How wide a net to cast when looking for a network. Each step up costs more
// transmit bursts, so we always start with the gentlest one that can work.
enum class SearchScope : uint8_t
{
  Cached,  // the remembered operator and band - fewest bursts
  LteOnly, // every LTE band, but no GSM
  Full     // everything the module supports
};

const char *scopeName(SearchScope scope)
{
  switch (scope)
  {
    case SearchScope::Cached: return "cached operator and band";
    case SearchScope::LteOnly: return "LTE only, all bands";
    default: return "all bands including GSM";
  }
}

uint32_t scopeTimeout(SearchScope scope)
{
  // Don't spend the whole budget on a narrow guess; the last step gets the rest.
  switch (scope)
  {
    case SearchScope::Cached: return 30000;
    case SearchScope::LteOnly: return 30000;
    default: return LTE_NETWORK_TIMEOUT_MS;
  }
}

// Must run with the radio parked (AT+CFUN=0) for the changes to take effect.
void applySearchScope(SearchScope scope, const NetworkCache &cache)
{
  // Capture the factory mask on every bring-up, not just when narrowing -
  // otherwise a device that never registers has nothing to widen back to.
  rememberFullBandScope();

  const String full = loadFullBandScope();

#if LTE_FORCE_BAND
  // A hard band pin overrides every scope - there is nothing else to try.
  {
    const String mask = lteBandMask(LTE_FORCE_BAND);
    if (mask.length() > 0 && full.length() > 0)
    {
      String wanted = csvField(full, 0) + "," + mask;
      const String tds = csvField(full, 2);
      if (tds.length() > 0)
        wanted += "," + tds;
      setIfDifferent("AT+CNBP?", "+CNBP:", "AT+CNBP=", wanted);
      setIfDifferent("AT+CNMP?", "+CNMP:", "AT+CNMP=", "38");
      Logger::log<Logger::Topic::LTE>("Band pinned to B{} by build flag\n", (int)LTE_FORCE_BAND);
      return;
    }
  }
#endif

  if (scope == SearchScope::Cached && cache.isValid())
  {
    if (cache.act == 7) // E-UTRAN
      setIfDifferent("AT+CNMP?", "+CNMP:", "AT+CNMP=", "38");

    const String mask = lteBandMask(cache.band);
    if (mask.length() > 0 && full.length() > 0)
    {
      // Keep the GSM and TD-SCDMA fields as they came, replace only the LTE one.
      String wanted = csvField(full, 0) + "," + mask;
      const String tds = csvField(full, 2);
      if (tds.length() > 0)
        wanted += "," + tds;
      setIfDifferent("AT+CNBP?", "+CNBP:", "AT+CNBP=", wanted);
    }
    return;
  }

  if (full.length() > 0)
    setIfDifferent("AT+CNBP?", "+CNBP:", "AT+CNBP=", full);

  setIfDifferent("AT+CNMP?", "+CNMP:", "AT+CNMP=", scope == SearchScope::LteOnly ? "38" : "2");
}

// Enabling the radio is the single biggest current step of the bring-up. Park it
// first, set the search scope while it is off, let the rail settle, and only
// then switch it on - that keeps both the number of transmit bursts and the
// chance of a brown-out to a minimum.
bool enableRadio(SearchScope scope, const NetworkCache &cache)
{
  writeCommand("AT+CFUN=0");
  waitResponse(10000, nullptr);

  applySearchScope(scope, cache);
  Logger::log<Logger::Topic::LTE>("Search scope: {}\n", scopeName(scope));

  phase("radio parked, letting the rail settle");
  delay(LTE_RADIO_SETTLE_MS);

  writeCommand("AT+CFUN=1");
  if (waitResponse(10000, nullptr) != AtResult::Ok)
    return false;

  // Operator selection must be automatic, and the mode is kept in the module's
  // NVM - a module left in manual mode reports CEREG stat 0 forever, because it
  // never starts searching. Only write it when it is wrong: AT+COPS=0 performs
  // the registration itself and can block for a minute.
  //
  // Forcing a specific operator with AT+COPS=4,2,"<numeric>" was tried here and
  // made things worse: it blocked for its full 60 s and failed, on the very cell
  // automatic selection then found straight away.
  String resp;
  writeCommand("AT+COPS?");
  if (waitResponse(5000, &resp) == AtResult::Ok && csvField(extractValue(resp, "+COPS:"), 0).toInt() != 0)
  {
    Logger::log<Logger::Topic::LTE>("Operator selection was not automatic, fixing\n");
    writeCommand("AT+COPS=0");
    waitResponse(60000, nullptr);
  }

  return true;
}

bool openIpStack()
{
  // Manual receive mode: nothing arrives on the UART until we ask for it with
  // AT+CIPRXGET=2, which keeps us in control of flow (no hardware RTS/CTS).
  writeCommand("AT+CIPRXGET=1");
  waitResponse(3000, nullptr);

  // Non-transparent (command) mode
  writeCommand("AT+CIPMODE=0");
  waitResponse(3000, nullptr);

  g_netopenSeen = false;
  g_netopenResult = -1;

  String resp;
  writeCommand("AT+NETOPEN");
  AtResult result = waitResponse(20000, &resp);

  const bool alreadyOpen = (result == AtResult::Error && resp.indexOf("already opened") >= 0);

  if (result == AtResult::Error && !alreadyOpen)
  {
    Logger::log<Logger::Level::ERROR, Logger::Topic::LTE>("NETOPEN failed: {}\n", resp);
    return false;
  }

  if (!alreadyOpen)
  {
    // Wait for the asynchronous "+NETOPEN: <err>". Polling AT+NETOPEN? here
    // instead would read this very URC as the query's answer - and because the
    // query means the opposite (0 = closed), it would look like a failure and
    // leave the real answer in the stream, shifting every response after it.
    uint32_t deadline = millis() + 30000;
    while (!g_netopenSeen && (int32_t)(millis() - deadline) < 0)
    {
      String line;
      if (!readLineRaw(line, 500) || line.length() == 0)
        continue;
      Logger::log<Logger::Level::DEBUG, Logger::Topic::LTE>("<< {}\n", line);
      handleUrc(line);
    }

    if (!g_netopenSeen)
    {
      Logger::log<Logger::Level::ERROR, Logger::Topic::LTE>("No NETOPEN confirmation\n");
      return false;
    }

    if (g_netopenResult != 0)
    {
      Logger::log<Logger::Level::ERROR, Logger::Topic::LTE>("NETOPEN failed with code {}\n", g_netopenResult);
      return false;
    }
  }

  writeCommand("AT+IPADDR");
  if (waitResponse(5000, &resp) == AtResult::Ok)
    g_ipAddress = extractValue(resp, "+IPADDR:");

  if (g_ipAddress.length() == 0)
  {
    Logger::log<Logger::Level::ERROR, Logger::Topic::LTE>("No IP address assigned\n");
    return false;
  }

  Logger::log<Logger::Topic::LTE>("IP address: {}\n", g_ipAddress);
  return true;
}

} // namespace

///////////////////////////////////////////////
// Public API
///////////////////////////////////////////////

bool begin()
{
  if (g_ready)
    return true;

  uint32_t started = millis();
  g_beginStart = started;
  g_moduleRestarts = 0;
  g_expectBootUrc = !rtc_modemPowered; // a cold start announces itself once

  // The control pins are latched (gpio_hold_en) across deep sleep. Drive the
  // output registers to the level they are currently held at BEFORE releasing
  // the latch - otherwise the pads would briefly float back to hi-Z and a
  // still-running modem would lose its supply.
  g_poweredOn = rtc_modemPowered;

  pinMode(LTE_PIN_PWR_EN, OUTPUT);
  digitalWrite(LTE_PIN_PWR_EN, g_poweredOn ? HIGH : LOW);
  pinMode(LTE_PIN_DTR, OUTPUT);
  digitalWrite(LTE_PIN_DTR, LOW); // DTR low keeps the module awake / wakes it up

  gpio_hold_dis((gpio_num_t)LTE_PIN_PWR_EN);
  gpio_hold_dis((gpio_num_t)LTE_PIN_DTR);
  gpio_deep_sleep_hold_dis();

  modem.setRxBufferSize(UART_RX_BUFFER);
  modem.begin(LTE_BOOT_BAUD, SERIAL_8N1, LTE_PIN_RX, LTE_PIN_TX);

  if (!g_poweredOn)
  {
    // Hold the rail down for a defined moment first - see LTE_POWER_SETTLE_MS
    applyPower(false);
    delay(LTE_POWER_SETTLE_MS);

    Logger::log<Logger::Topic::LTE>("Powering up modem (EN high)...\n");
    applyPower(true);
  }
  else
  {
    // Kept powered across deep sleep: give it a moment to wake from CSCLK sleep
    Logger::log<Logger::Topic::LTE>("Modem still powered, waking up...\n");
    delay(100);
  }

  // Battery voltage before we ask the modem for anything: a bring-up that fails
  // this early is almost always the supply, and this is the one number that says
  // so even when the modem never answers.
  Logger::log<Logger::Topic::LTE>("VBAT at power-up: {} V\n", String(Board::getBatteryVoltage(), 2));

  if (!syncAt(LTE_BOOT_TIMEOUT_MS))
  {
    Logger::log<Logger::Level::ERROR, Logger::Topic::LTE>("Modem did not respond to AT\n");
    return false;
  }
  phase("module booted, AT responding");

  // Let the module finish its own boot before we configure anything.
  waitForBootQuiet(1500, 15000);
  g_expectBootUrc = false; // from here on, a boot URC means the module restarted
  phase("boot chatter settled");

  writeCommand("ATE0"); // echo off - we parse responses, not our own commands
  waitResponse(2000, nullptr);
  drainInput();

  writeCommand("AT+CMEE=2"); // verbose errors
  waitResponse(2000, nullptr);

  switchBaud();

  writeCommand("AT+CSCLK=0"); // no UART sleep while we're working
  waitResponse(2000, nullptr);
  phase("UART configured");

  if (!waitForSim(15000))
    return false;
  phase("SIM ready");

  readImei();
  forgetOtherModule();

  // A configured APN goes in before registration so the initial attach already
  // uses the right context; otherwise we ask the network afterwards.
  String configuredApn = loadStoredApn();
  if (configuredApn.length() == 0)
    configuredApn = LTE_APN;

  if (configuredApn.length() > 0)
    setApn(configuredApn);

  NetworkCache cache = loadNetworkCache();

  // Escalation ladder, gentlest first.
  SearchScope ladder[3];
  uint8_t steps = 0;
  if (cache.isValid())
    ladder[steps++] = SearchScope::Cached;
#if LTE_PREFER_LTE_ONLY
  ladder[steps++] = SearchScope::LteOnly;
#endif
  ladder[steps++] = SearchScope::Full;

  bool registered = false;
  for (uint8_t step = 0; step < steps && !registered; step++)
  {
    if (!enableRadio(ladder[step], cache))
    {
      Logger::log<Logger::Level::ERROR, Logger::Topic::LTE>("Could not enable the radio\n");
      continue;
    }

    registered = waitForRegistration(scopeTimeout(ladder[step]));

    if (!registered && ladder[step] == SearchScope::Cached)
    {
      // Moved, or the cell changed band. Drop the cache and widen.
      Logger::log<Logger::Level::WARNING, Logger::Topic::LTE>("Cached network unreachable, clearing cache\n");
      clearNetworkCache();
      cache = NetworkCache{};
    }
  }

  if (!registered)
  {
    Logger::log<Logger::Level::ERROR, Logger::Topic::LTE>("No network found at any search scope\n");
    return false;
  }
  phase("registered to network");

  if (configuredApn.length() == 0)
  {
    String networkApn = queryNetworkApn();
    if (networkApn.length() > 0)
    {
      Logger::log<Logger::Topic::LTE>("Network supplied APN: {}\n", networkApn);
      setApn(networkApn);
    }
    else
    {
      Logger::log<Logger::Level::WARNING, Logger::Topic::LTE>(
        "No APN configured and network did not supply one, using SIM default\n");
    }
  }
  else
  {
    Logger::log<Logger::Topic::LTE>("Using APN: {}\n", configuredApn);
  }

  if (!waitForAttach(LTE_NETWORK_TIMEOUT_MS))
    return false;
  phase("PDP attached");

  readSignalQuality();
  readIdentity();

  if (!openIpStack())
    return false;

  g_ready = true;
  Logger::log<Logger::Topic::LTE>("Modem ready in {} ms (operator: {})\n", millis() - started,
                                  g_operator.length() ? g_operator : String("unknown"));
  return true;
}

bool isReady() { return g_ready; }

void shutdown()
{
  if (!g_poweredOn)
    return;

  if (g_sockOpen)
    socketClose();

  if (g_ready)
  {
    writeCommand("AT+NETCLOSE");
    waitResponse(10000, nullptr);
    drainUrcs(1000); // swallow the late "+NETCLOSE: 0"
  }

  g_ready = false;

  #ifdef LTE_KEEP_POWERED
  // Keep the registration, park the module in low power UART sleep.
  writeCommand("AT+CSCLK=1");
  waitResponse(2000, nullptr);
  digitalWrite(LTE_PIN_DTR, HIGH); // DTR high -> module sleeps
  Logger::log<Logger::Topic::LTE>("Modem parked in sleep mode (DTR high)\n");
  #else
  // Graceful detach + power down. "OK" comes back before the module has
  // finished writing its state; cutting the supply in that window can corrupt
  // the module's flash (A7682E manual 3.2.1), so give it the ~2 s it needs.
  writeCommand("AT+CPOF");
  waitResponse(5000, nullptr);
  delay(LTE_POWER_OFF_GRACE_MS);
  modem.end();
  applyPower(false);
  Logger::log<Logger::Topic::LTE>("Modem powered off (EN low)\n");
  #endif
}

void prepareForDeepSleep()
{
  shutdown();

  // The ESP32 tri-states its pads in deep sleep, so latch both control lines at
  // their current level. The boost enable has a pull-down and would default to
  // off, but LTE_KEEP_POWERED needs it held high, and DTR must not drift and
  // wake/sleep the module at random.
  // Set the output registers first, then re-latch: if a hold was still active
  // (e.g. we never brought the modem up this cycle) releasing it before the
  // registers are valid would let the pads float for a moment.
  pinMode(LTE_PIN_PWR_EN, OUTPUT);
  pinMode(LTE_PIN_DTR, OUTPUT);
  #ifdef LTE_KEEP_POWERED
  digitalWrite(LTE_PIN_PWR_EN, HIGH);
  digitalWrite(LTE_PIN_DTR, HIGH); // DTR high -> module stays in CSCLK sleep
  #else
  digitalWrite(LTE_PIN_PWR_EN, LOW);
  digitalWrite(LTE_PIN_DTR, LOW);
  #endif

  gpio_hold_dis((gpio_num_t)LTE_PIN_PWR_EN);
  gpio_hold_dis((gpio_num_t)LTE_PIN_DTR);
  gpio_hold_en((gpio_num_t)LTE_PIN_PWR_EN);
  gpio_hold_en((gpio_num_t)LTE_PIN_DTR);
  gpio_deep_sleep_hold_en();
}

int8_t getRssi() { return g_rssi; }

const String &getOperator() { return g_operator; }

const String &getImei() { return g_imei; }

const String &getIpAddress() { return g_ipAddress; }

const String &getApn() { return g_apn; }

void setStoredApn(const String &apn)
{
  Preferences prefs;
  prefs.begin(NVS_NAMESPACE);
  if (apn.length() > 0)
    prefs.putString("apn", apn);
  else
    prefs.remove("apn");
  prefs.end();
  Logger::log<Logger::Topic::LTE>("Stored APN: {}\n", apn.length() ? apn : String("<cleared>"));
}

void clearStoredApn() { setStoredApn(""); }

void saveNetworkCache()
{
  NetworkCache cache;
  String resp;

  // Numeric format, so the value can be fed straight back to AT+COPS=4,2,...
  writeCommand("AT+COPS=3,2");
  waitResponse(3000, nullptr);

  writeCommand("AT+COPS?");
  if (waitResponse(5000, &resp) == AtResult::Ok)
  {
    String value = extractValue(resp, "+COPS:");
    cache.oper = csvField(value, 2);
    cache.act = csvField(value, 3).toInt();
  }

  writeCommand("AT+CPSI?");
  if (waitResponse(5000, &resp) == AtResult::Ok)
  {
    int idx = resp.indexOf("EUTRAN-BAND");
    if (idx >= 0)
      cache.band = resp.substring(idx + 11).toInt();
  }

  if (!cache.isValid() || cache.band == 0)
  {
    Logger::log<Logger::Level::WARNING, Logger::Topic::LTE>("Nothing to cache (operator '{}', band {})\n",
                                                            cache.oper, cache.band);
    return;
  }

  NetworkCache stored = loadNetworkCache();
  if (stored.oper == cache.oper && stored.act == cache.act && stored.band == cache.band)
    return;

  storeNetworkCache(cache);
  Logger::log<Logger::Topic::LTE>("Cached network: {} (act {}, band {})\n", cache.oper, cache.act, cache.band);
}

void invalidateNetworkCache()
{
  clearNetworkCache();
  Logger::log<Logger::Topic::LTE>("Network cache cleared, next bring-up will search all bands\n");
}

///////////////////////////////////////////////
// Socket
///////////////////////////////////////////////

bool socketOpen(const char *host, uint16_t port)
{
  if (!g_ready)
    return false;

  if (g_sockOpen)
    socketClose();

  g_sockRemoteClosed = false;
  g_sockDataPending = false;
  g_sockPendingRest = 0;
  g_cipopenSeen = false;
  g_cipopenResult = -1;

  char cmd[160];
  snprintf(cmd, sizeof(cmd), "AT+CIPOPEN=0,\"TCP\",\"%s\",%u", host, (unsigned)port);
  writeCommand(cmd);

  // "OK" acknowledges the command; the connection result arrives as +CIPOPEN URC
  if (waitResponse(5000, nullptr) != AtResult::Ok && !g_cipopenSeen)
  {
    Logger::log<Logger::Level::ERROR, Logger::Topic::LTE>("CIPOPEN rejected\n");
    return false;
  }

  uint32_t deadline = millis() + 30000;
  while (!g_cipopenSeen && (int32_t)(millis() - deadline) < 0)
  {
    String line;
    if (readLineRaw(line, 500) && line.length() > 0)
    {
      Logger::log<Logger::Level::DEBUG, Logger::Topic::LTE>("<< {}\n", line);
      handleUrc(line);
    }
  }

  if (!g_cipopenSeen || g_cipopenResult != 0)
  {
    Logger::log<Logger::Level::ERROR, Logger::Topic::LTE>("Connect to {}:{} failed (result {})\n", host, port,
                                                          g_cipopenResult);
    socketClose();
    return false;
  }

  g_sockOpen = true;
  Logger::log<Logger::Topic::LTE>("Connected to {}:{}\n", host, port);
  return true;
}

void socketClose()
{
  if (!g_poweredOn)
    return;

  writeCommand("AT+CIPCLOSE=0");
  waitResponse(10000, nullptr); // ERROR when already closed - harmless
  drainUrcs(1000);              // swallow the late "+CIPCLOSE: 0,0"

  g_sockOpen = false;
  g_sockRemoteClosed = false;
  g_sockDataPending = false;
  g_sockPendingRest = 0;
}

bool socketIsOpen() { return g_sockOpen; }

bool socketRemoteClosed() { return g_sockRemoteClosed; }

bool socketHasPendingData() { return g_sockDataPending || g_sockPendingRest > 0; }

int socketSend(const uint8_t *data, size_t len)
{
  if (!g_sockOpen || len == 0)
    return -1;

  size_t sent = 0;
  while (sent < len)
  {
    size_t chunk = min(len - sent, (size_t)LTE_IO_CHUNK);

    char cmd[32];
    snprintf(cmd, sizeof(cmd), "AT+CIPSEND=0,%u", (unsigned)chunk);
    writeCommand(cmd);

    if (!waitForPrompt(10000))
    {
      Logger::log<Logger::Level::ERROR, Logger::Topic::LTE>("No '>' prompt for CIPSEND\n");
      return sent > 0 ? (int)sent : -1;
    }

    g_cipsendSeen = false;
    g_cipsendConfirmed = 0;

    modem.write(data + sent, chunk);
    modem.flush();

    if (waitResponse(30000, nullptr) != AtResult::Ok)
    {
      Logger::log<Logger::Level::ERROR, Logger::Topic::LTE>("CIPSEND failed after {} bytes\n", sent);
      return sent > 0 ? (int)sent : -1;
    }

    // "OK" only means the module accepted the payload. Wait for the "+CIPSEND:"
    // confirmation too - leaving it in the stream would desynchronise every
    // response that follows.
    uint32_t deadline = millis() + 30000;
    while (!g_cipsendSeen && (int32_t)(millis() - deadline) < 0)
    {
      String line;
      if (!readLineRaw(line, 500) || line.length() == 0)
        continue;
      Logger::log<Logger::Level::DEBUG, Logger::Topic::LTE>("<< {}\n", line);
      handleUrc(line);
    }

    if (!g_cipsendSeen)
    {
      Logger::log<Logger::Level::ERROR, Logger::Topic::LTE>("No CIPSEND confirmation after {} bytes\n", sent);
      return sent > 0 ? (int)sent : -1;
    }

    if (g_cipsendConfirmed != (int)chunk)
    {
      Logger::log<Logger::Level::ERROR, Logger::Topic::LTE>("CIPSEND sent only {} of {} bytes\n", g_cipsendConfirmed,
                                                            chunk);
      return (int)(sent + g_cipsendConfirmed);
    }

    sent += chunk;
  }

  return (int)sent;
}

int socketBufferedLength()
{
  if (!g_sockOpen)
    return -1;

  String resp;
  writeCommand("AT+CIPRXGET=4,0");
  if (waitResponse(5000, &resp) != AtResult::Ok)
    return -1;

  String value = extractValue(resp, "+CIPRXGET:");
  if (value.length() == 0)
    return -1;

  return csvField(value, 2).toInt();
}

int socketRecv(uint8_t *buf, size_t maxLen, int *rest)
{
  if (rest)
    *rest = 0;

  if (!g_sockOpen)
    return -1;

  if (maxLen > LTE_IO_CHUNK)
    maxLen = LTE_IO_CHUNK;

  char cmd[40];
  snprintf(cmd, sizeof(cmd), "AT+CIPRXGET=2,0,%u", (unsigned)maxLen);
  writeCommand(cmd);

  // Locate the "+CIPRXGET: 2,<id>,<len>,<rest>" header that precedes the payload
  String header;
  bool found = false;
  bool noData = false;
  bool failed = false;

  uint32_t deadline = millis() + 15000;
  while ((int32_t)(millis() - deadline) < 0)
  {
    String line;
    if (!readLineRaw(line, 2000))
      break;
    if (line.length() == 0)
      continue;

    Logger::log<Logger::Level::DEBUG, Logger::Topic::LTE>("<< {}\n", line);

    if (line.startsWith("+CIPRXGET: 2"))
    {
      header = line;
      found = true;
      break; // the raw payload follows immediately
    }

    // "+IP ERROR: No data" is not a failure - the modem's receive buffer is
    // simply empty at this instant. It is followed by a plain "ERROR", which
    // must be consumed as well or every later response shifts by one line.
    if (line.indexOf("No data") >= 0)
    {
      noData = true;
      continue;
    }

    if (isFinalOk(line))
      continue;

    if (isFinalError(line))
    {
      failed = !noData;
      break; // final result code consumed - the stream stays in sync
    }

    handleUrc(line);
  }

  if (!found)
  {
    if (noData)
    {
      // Nothing buffered right now; the caller backs off and polls again.
      if (g_sockRemoteClosed)
      {
        int buffered = socketBufferedLength();
        Logger::log<Logger::Level::DEBUG, Logger::Topic::LTE>(
          "No data after peer close, modem still holds {} bytes\n", buffered);
      }
      g_sockDataPending = false;
      g_sockPendingRest = 0;
      return 0;
    }

    if (failed)
      Logger::log<Logger::Level::WARNING, Logger::Topic::LTE>("CIPRXGET rejected\n");
    else
      Logger::log<Logger::Level::WARNING, Logger::Topic::LTE>("CIPRXGET: no data header\n");
    return -1;
  }

  String value = header.substring(header.indexOf(": ") + 2);
  int readLen = csvField(value, 2).toInt();
  int restLen = csvField(value, 3).toInt();

  g_sockPendingRest = restLen;
  if (rest)
    *rest = restLen;

  size_t got = 0;
  if (readLen > 0)
  {
    got = readRaw(buf, (size_t)readLen, 15000);
    if (got != (size_t)readLen)
      Logger::log<Logger::Level::WARNING, Logger::Topic::LTE>("Short read: got {} of {} bytes\n", got, readLen);
  }

  // Consume the trailing OK
  waitResponse(5000, nullptr);

  // The modem only re-raises "+CIPRXGET: 1" when new data arrives, so track
  // the buffered remainder ourselves.
  g_sockDataPending = restLen > 0;

  return (int)got;
}

void pumpUrc()
{
  while (modem.available())
  {
    String line;
    if (!readLineRaw(line, 50))
      break;
    if (line.length() == 0)
      continue;
    Logger::log<Logger::Level::DEBUG, Logger::Topic::LTE>("<< {}\n", line);
    handleUrc(line);
  }
}

AtResult sendCommand(const char *cmd, uint32_t timeoutMs, String *response)
{
  writeCommand(cmd);
  return waitResponse(timeoutMs, response);
}

} // namespace LteModem

#endif // USE_LTE_MODEM
