#ifndef LTE_MODEM_H
#define LTE_MODEM_H

#ifdef USE_LTE_MODEM

  #include <Arduino.h>

///////////////////////////////////////////////
// Wiring (A7682E carrier on the Paperboard's U11)
//
// The carrier mates with the Paperboard's U11 (SH 1.0 mm, 6 pin) through a
// straight 1:1 cable. The Paperboard routes that connector to the silkscreen
// labels D9..D12, which are ESP32-S3 GPIOs 10..13 - the upper half of the epdiy
// parallel bus, unused by 8-bit displays.
//
//   J3.3 / D9  / GPIO10  MCU_RX  <- modem TX
//   J3.4 / D10 / GPIO11  MCU_TX  -> modem RX
//   J3.5 / D11 / GPIO12  DTR     (LOW = keep awake, HIGH = allow sleep)
//   J3.6 / D12 / GPIO13  EN      (HIGH = modem powered, LOW = hard off)
//
// Two carriers share this connector and these semantics:
//
//   lte-a7682e-carrier   the A7682E sits on the board itself. EN drives a load
//                        switch onto the module's VBAT, the board pulses PWRKEY
//                        by itself on power-up, and a level shifter bridges the
//                        module's 1.8 V UART to 3.3 V. Bulk sits right at VBAT.
//   lte-a7682e-daughter  the earlier board: EN enables a TPS61022 5 V boost that
//                        feeds a third-party carrier. Marginal - attach can
//                        brown it out unless the 5 V rail carries ~2 mF of bulk.
//
// Both pull EN down, so a floating pad (MCU in reset, cable unplugged) leaves
// the modem off.
///////////////////////////////////////////////

  #ifndef LTE_PIN_RX
    #define LTE_PIN_RX 10
  #endif
  #ifndef LTE_PIN_TX
    #define LTE_PIN_TX 11
  #endif
  #ifndef LTE_PIN_PWR_EN
    #define LTE_PIN_PWR_EN 13
  #endif
  #ifndef LTE_PIN_DTR
    #define LTE_PIN_DTR 12
  #endif

  // UART port used for the modem (0 is the USB/debug console)
  #ifndef LTE_UART_NUM
    #define LTE_UART_NUM 1
  #endif

  // Baud rate used after the initial handshake. AT+IPR can raise it so a bulk
  // image download isn't UART bound, but it is not worth the risk by default:
  // one A7682E sample took 460800 happily (1.3 s for a 30 kB image), another
  // rebooted on the AT+IPR itself, which costs ~12 s of recovery and loses the
  // ATE0/CMEE setup. At 115200 the same image takes about 4 s, which is nothing
  // against a refresh interval of minutes. Raise it only if your module is
  // known to cope. Equal to LTE_BOOT_BAUD = no switch at all.
  #ifndef LTE_BAUD
    #define LTE_BAUD 115200
  #endif
  #ifndef LTE_BOOT_BAUD
    #define LTE_BOOT_BAUD 115200
  #endif

  // APN override. Empty = ask the network (AT+CGNAPN) / rely on the SIM default.
  // An APN stored in NVS (key "apn") takes precedence over this.
  #ifndef LTE_APN
    #define LTE_APN ""
  #endif

  // How long to wait for network registration + PDP attach
  #ifndef LTE_NETWORK_TIMEOUT_MS
    #define LTE_NETWORK_TIMEOUT_MS 60000
  #endif

  // How long to wait for the module to answer AT after power is applied
  #ifndef LTE_BOOT_TIMEOUT_MS
    #define LTE_BOOT_TIMEOUT_MS 25000
  #endif

  // Quiet time with the radio parked (AT+CFUN=0) before the first search.
  // Enabling the radio is the biggest current step in the whole bring-up, so on
  // a marginal 5 V rail it pays to meet it with a settled converter and a fully
  // topped-up bulk capacitor.
  #ifndef LTE_RADIO_SETTLE_MS
    #define LTE_RADIO_SETTLE_MS 1500
  #endif

  // Prefer LTE-only on a cold search (AT+CNMP=38). GSM's TDMA transmit bursts
  // are the harshest current events the supply has to survive, and skipping
  // them makes a marginal 5 V rail far more likely to make it through attach.
  // Set to 0 to always allow the GSM fallback.
  #ifndef LTE_PREFER_LTE_ONLY
    #define LTE_PREFER_LTE_ONLY 1
  #endif

  // The module's complete band mask, written whenever the search is widened.
  // It must be a known constant rather than read back from the module: AT+CNBP
  // is persistent in the module's NVM and this firmware narrows it itself, so a
  // value read back later may already be narrowed - widening to it would pin
  // the device to one band for good. This is what two A7682E-LASA samples
  // reported before anything was written to them (GSM, LTE, TD-SCDMA masks).
  #ifndef LTE_FULL_BAND_SCOPE
    #define LTE_FULL_BAND_SCOPE "0X0000000000000180,0X00000000000800C5,0X0000"
  #endif

  // Pin the LTE search to one band. A low band (20 = 800 MHz, 8 = 900 MHz) has
  // far less path loss than 2100 MHz, so the network commands a lower transmit
  // power - which directly shrinks the current burst a weak supply has to take.
  // Useful for a fixed installation; leave at 0 to search all bands.
  #ifndef LTE_FORCE_BAND
    #define LTE_FORCE_BAND 0
  #endif

  // Extra quiet time after the module has browned out, to let the rail recover
  // before we let it transmit again.
  #ifndef LTE_RESET_BACKOFF_MS
    #define LTE_RESET_BACKOFF_MS 2500
  #endif

  // Time between AT+CPOF and actually cutting the supply. The module answers
  // OK before it has finished saving its state, and a hard power cut in that
  // window can corrupt its flash.
  #ifndef LTE_POWER_OFF_GRACE_MS
    #define LTE_POWER_OFF_GRACE_MS 2000
  #endif

  // EN low time before a cold start, so the modem gets a clean power-on reset
  // rather than coming up half-enabled and never answering AT.
  //
  // This is only a guarantee where the supply is actively discharged when EN
  // drops - the integrated carrier's load switch does that. On the daughter
  // board the rail is bled only by the 3.8EN divider, its time constant is tens
  // of seconds, and 300 ms does NOT reset a module that is still running (e.g.
  // after an ESP reset without deep sleep). In the normal flow the modem has
  // been off for a whole sleep interval, so either board starts cold.
  #ifndef LTE_POWER_SETTLE_MS
    #define LTE_POWER_SETTLE_MS 300
  #endif

  // Largest chunk requested from the modem in one AT+CIPRXGET / sent in one
  // AT+CIPSEND. The A76xx socket API caps both at 1500 bytes.
  #ifndef LTE_IO_CHUNK
    #define LTE_IO_CHUNK 1460
  #endif

namespace LteModem
{

enum class AtResult : uint8_t
{
  Ok,
  Error,
  Timeout
};

// --- Lifecycle -------------------------------------------------------------

// Power up the module, sync AT, unlock SIM, register, attach PDP and open the
// TCP/IP stack. Returns true once a socket can be opened.
bool begin();

// True if begin() completed and the IP stack is up.
bool isReady();

// Release the network. With LTE_KEEP_POWERED the module is put into AT+CSCLK
// sleep (registration is retained), otherwise it is powered down completely.
void shutdown();

// Last-chance cleanup before esp_deep_sleep_start(): make sure the module is
// off/asleep and latch the control pins so they don't float while the ESP32
// tri-states its pads.
void prepareForDeepSleep();

// --- Status ----------------------------------------------------------------

int8_t getRssi(); // dBm, 0 if unknown
const String &getOperator();
const String &getImei();
const String &getIpAddress();
const String &getApn();

// APN stored in NVS (overrides the LTE_APN build flag); "" clears it.
void setStoredApn(const String &apn);
void clearStoredApn();

// Remembers the operator, radio access technology and band we just registered
// on, so the next bring-up can go straight there instead of sweeping every
// band - fewer transmit bursts to survive, and a faster attach.
void saveNetworkCache();
void invalidateNetworkCache();

// --- TCP socket (single connection, id 0) ----------------------------------

bool socketOpen(const char *host, uint16_t port);
void socketClose();
bool socketIsOpen();
bool socketRemoteClosed();

// Returns the number of bytes handed to the modem, or -1 on failure.
int socketSend(const uint8_t *data, size_t len);

// Reads up to maxLen bytes. Returns the byte count (0 = nothing available
// right now), or -1 on failure. *rest, if given, receives the number of bytes
// the modem still has buffered for us.
int socketRecv(uint8_t *buf, size_t maxLen, int *rest = nullptr);

// True if the modem signalled (or we know of) buffered incoming data.
bool socketHasPendingData();

// Bytes the modem still holds for us (AT+CIPRXGET=4), or -1 if unknown.
int socketBufferedLength();

// Drain and dispatch any unsolicited result codes without blocking.
void pumpUrc();

// --- Raw AT access (diagnostics / OTA helpers) -----------------------------

AtResult sendCommand(const char *cmd, uint32_t timeoutMs = 2000, String *response = nullptr);

} // namespace LteModem

#endif // USE_LTE_MODEM
#endif // LTE_MODEM_H
