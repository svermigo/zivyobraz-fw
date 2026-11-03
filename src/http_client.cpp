#include "http_client.h"

#include "board.h"
#include "sensor.h"
#include "display.h"
#include "logger.h"
#include "state_manager.h"
#include "wireless.h"

#ifdef USE_LTE_MODEM
  #include "lte_modem.h"

  #include <Update.h>
#else
#include <HTTPUpdate.h>
#endif

// External configuration
extern const char *host;
extern const char *firmware;

// Timeouts for HTTP operations
static constexpr uint32_t TOTAL_TIMEOUT_MS = 30000;
static constexpr uint32_t IDLE_TIMEOUT_MS = 5000;

HttpClient::HttpClient()
    : m_sleepDuration(StateManager::DEFAULT_SLEEP_SECONDS),
      m_serverTimestamp(0),
      m_displayRotation(0),
      m_hasRotation(false),
      m_partialRefresh(false),
      m_otaRequired(false),
      m_otaUrl(""),
      m_imageDataReady(false),
      m_jsonPayload("")
{
#ifdef USE_CLIENT_HTTPS
  // Configure secure connection - only once in constructor
  m_client.setInsecure();
  m_client.setTimeout(15); // Timeout for TLS handshake and read operations
#endif
}

void HttpClient::buildJsonPayload()
{
  // Only build once - if already built, skip
  if (m_jsonPayload.length() > 0)
    return;

  // API and firmware info
  m_jsonDoc["fwVersion"] = firmware;
  m_jsonDoc["apiVersion"] = firmware;  // tells server what are firmware capabilities, same as fwVersion in our case
  m_jsonDoc["buildDate"] = BUILD_DATE; // got from scripts/set_build_date.py before build
  m_jsonDoc["board"] = Board::getBoardType();

  // System info
  JsonObject system = m_jsonDoc["system"].to<JsonObject>();
  system["cpuTemp"] = Board::getCPUTemperature();
  system["resetReason"] = Board::getResetReasonString();
  system["vccVoltage"] = Board::getBatteryVoltage();

  // Network info
  JsonObject network = m_jsonDoc["network"].to<JsonObject>();
#ifdef USE_LTE_MODEM
  // "ssid" carries the network identity the device is attached to; on cellular
  // that is the operator, with the extra modem details alongside it.
  network["ssid"] = LteModem::getOperator();
  network["type"] = "lte";
  network["imei"] = LteModem::getImei();
  network["apn"] = LteModem::getApn();
#else
  network["ssid"] = WiFi.SSID();
#endif
  network["rssi"] = Wireless::getStrength();
  network["mac"] = Wireless::getMacAddress();
  network["apRetries"] = StateManager::getFailureCount();
  network["ipAddress"] = Wireless::getIPAddress();
  // Add last download duration if available from previous run (in milliseconds)
  if (StateManager::getLastDownloadDuration() > 0)
    network["lastDownloadDuration"] = StateManager::getLastDownloadDuration();

  // Display info
  JsonObject display = m_jsonDoc["display"].to<JsonObject>();
  display["type"] = Display::getDisplayType();
  display["width"] = Display::getResolutionX();
  display["height"] = Display::getResolutionY();
  display["colorType"] = Display::getColorType();
  // Add last refresh duration if available from previous run (in milliseconds)
  if (StateManager::getLastRefreshDuration() > 0)
    display["lastRefreshDuration"] = StateManager::getLastRefreshDuration();

#ifdef SENSOR
  // Add sensor data if available
  SensorData sensorData = Sensor::getInstance().getSensorData();
  if (sensorData.isValid)
  {
    JsonArray sensors = m_jsonDoc["sensors"].to<JsonArray>();
    sensorData.toJson(sensors);
  }
#endif

  serializeJson(m_jsonDoc, m_jsonPayload);
}

bool HttpClient::sendRequest(bool timestampCheck)
{
  // Build JSON payload (only built once, cached for subsequent requests)
  buildJsonPayload();

  Logger::log<Logger::Level::DEBUG, Logger::Topic::HTTP>("Connecting to: {}\n", host);

  // Try to connect with retries
  for (uint8_t attempt = 0; attempt < 3; attempt++)
  {
    if (m_client.connect(host, CONNECTION_PORT))
      break;

    Logger::log<Logger::Level::ERROR, Logger::Topic::HTTP>("Connection failed, retrying... {}/3\n", attempt + 1);
    if (attempt == 2)
    {
      m_sleepDuration = StateManager::DEFAULT_SLEEP_SECONDS;
      if (!timestampCheck)
        return false; // For image download, fail immediately
      delay(500);
    }
    if (!timestampCheck)
      delay(200);
  }

  // Send HTTP/HTTPS POST request with JSON body
  Logger::log<Logger::Level::DEBUG, Logger::Topic::HTTP>("Sending POST to: {}{}/index.php\n", CONNECTION_URL_PREFIX,
                                                         host);

  // Pretty print JSON payload for device info
  String prettyJson;
  serializeJsonPretty(m_jsonDoc, prettyJson);
  Logger::log<Logger::Level::INFO, Logger::Topic::HTTP>("JSON Payload:\n{}\n", prettyJson);

  // Build URL with timestampCheck query parameter
  String url = "/index.php?timestampCheck=";
  url += timestampCheck ? "1" : "0";

  // Send HTTP request using multiple print() calls to avoid allocating one large heap String
  m_client.print("POST ");
  m_client.print(url);
  m_client.print(" HTTP/1.1\r\nHost: ");
  m_client.print(host);
  m_client.print("\r\nX-API-Key: ");
  m_client.print(Utils::getStoredAPIKey());
  m_client.print("\r\nContent-Type: application/json\r\nContent-Length: ");
  m_client.print(m_jsonPayload.length());
  m_client.print("\r\nConnection: close\r\n\r\n");
  m_client.print(m_jsonPayload);

  Logger::log<Logger::Topic::HTTP>("Request sent\n");

  // Wait for response with timeout
  uint32_t timeout = millis();
  while (m_client.available() == 0)
  {
    if (millis() - timeout > 10000)
    {
      Logger::log<Logger::Level::WARNING, Logger::Topic::HTTP>(">>> Client Timeout!\n");
      m_client.stop();
      if (timestampCheck)
        m_sleepDuration = StateManager::DEFAULT_SLEEP_SECONDS;
      return false;
    }
  }

  return true;
}

bool HttpClient::parseHeaders(bool checkTimestampOnly, uint64_t storedTimestamp)
{
  bool connectionOk = false;
  bool foundTimestamp = false;
  m_hasRotation = false;
  m_partialRefresh = false;
  m_otaRequired = false;
  m_otaUrl = "";

  while (m_client.connected() || m_client.available())
  {
    String line = m_client.readStringUntil('\n');

    // Parse headers only when checking timestamp
    if (checkTimestampOnly)
    {
      // Parse X-OTA-Update header for firmware update
      if (line.startsWith("X-OTA-Update"))
      {
        m_otaUrl = line.substring(14); // Skip "X-OTA-Update: "
        m_otaUrl.trim();               // Remove whitespace and \r
        m_otaRequired = true;
        Logger::log<Logger::Topic::HEADER>("OTA Update requested: {}\n", m_otaUrl);
      }

      // Parse Timestamp header
      if (line.startsWith("Timestamp"))
      {
        foundTimestamp = true;
        // Skipping also colon and space - also in following code for sleep, rotate, ...
        m_serverTimestamp = line.substring(11).toInt();
        Logger::log<Logger::Topic::HEADER>("Timestamp now: {}\n", m_serverTimestamp);
      }

      // Is there another header (after the Sleep one) with sleep in Seconds?
      if (line.startsWith("PreciseSleep"))
      {
        m_sleepDuration = line.substring(14).toInt();
        Logger::log<Logger::Topic::HEADER>("Precise Sleep in seconds: {}\n", m_sleepDuration);
      }

      // Do we want to rotate display? (IE. upside down)
      if (line.startsWith("Rotate"))
      {
        m_displayRotation = line.substring(8).toInt();
        m_hasRotation = true;
        Logger::log<Logger::Topic::HEADER>("Rotation: {}\n", m_displayRotation);
      }

      // Partial refresh request from server (only if this line exists)
      if (line.startsWith("PartialRefresh"))
      {
        m_partialRefresh = true;
        Logger::log<Logger::Topic::HEADER>("Partial refresh requested\n");
      }

      // Server requests a full Wi-Fi scan next time - invalidate cached BSSID/channel
      // so the next boot performs an all-channel scan and connects to the strongest AP.
      if (line.startsWith("ForceWifiFullScan"))
      {
        Wireless::invalidateConnectionCache();
        Logger::log<Logger::Topic::HEADER>("Forced Wi-Fi full scan requested, cache invalidated\n");
      }

      // ShowNoWifiError setting (1 = show error on display, 0 = keep existing content)
      if (line.startsWith("ShowNoWifiError"))
      {
        uint8_t showNoWifiError = line.substring(17).toInt(); // Skip "ShowNoWifiError: "
        StateManager::setShowNoWifiError(showNoWifiError);
        Logger::log<Logger::Topic::HEADER>("ShowNoWifiError: {}\n", showNoWifiError);
      }
    }

#ifdef USE_LTE_MODEM
    // Log every header line, not just the status line - on a cellular link the
    // full block is what you need when the server answers something unexpected.
    Logger::log<Logger::Level::DEBUG, Logger::Topic::HTTP>("{}\n", line);
#endif

    // Check for successful HTTP response (always check)
    if (!connectionOk)
    {
      // Support both HTTP/1.0 and HTTP/1.1
      connectionOk = line.startsWith("HTTP/1.1 200 OK") || line.startsWith("HTTP/1.0 200 OK");
#ifndef USE_LTE_MODEM
      Logger::log<Logger::Level::DEBUG, Logger::Topic::HTTP>("{}\n", line);
#endif
    }

    // End of headers
    if (line == "\r")
    {
      Logger::log<Logger::Topic::HTTP>("Headers received\n");
      break;
    }
  }

  // Is there a problem? Fallback to default deep sleep time to try again soon
  if (!connectionOk)
  {
    m_sleepDuration = StateManager::DEFAULT_SLEEP_SECONDS;
    return false;
  }

  // For debug purposes - print out the whole response
  /*
  String data = "";
  while (m_client.connected() || m_client.available()) {
    if (m_client.available()) {
      data += m_client.read();  // Read one byte
    }
  }
  m_client.stop();
  Logger::log<Logger::Level::DEBUG, Logger::Topic::HTTP>("Byte by byte:{}\n", data);
  /* */

  // If checking timestamp, see if content changed
  if (checkTimestampOnly)
  {
    // Always save sleep duration from server, even if no update is needed
    StateManager::setSleepDuration(m_sleepDuration);

#ifdef USE_LTE_MODEM
    // A response without the Timestamp header carries no image - the server
    // answers this way when there is nothing new to draw. Falling through would
    // report an update, find no image data, and then still refresh the display
    // from the empty framebuffer, wiping whatever is on screen.
    if (!foundTimestamp && !m_otaRequired)
    {
      Logger::log<Logger::Level::WARNING, Logger::Topic::HTTP>(
        "No Timestamp header in response, keeping current screen\n");

      StateManager::setLastRefreshDuration(0);

      return false;
    }
#endif

    if (foundTimestamp && (m_serverTimestamp == storedTimestamp))
    {
      Logger::log<Logger::Topic::HTTP>("No screen reload, still at current timestamp: {}\n", storedTimestamp);

      StateManager::setLastRefreshDuration(0);

      return false;
    }

    // Set timestamp to actual one
    StateManager::setTimestamp(m_serverTimestamp);
  }

  return true;
}

bool HttpClient::checkForUpdate(bool timestampCheck, bool keepConnectionOpen)
{
  m_imageDataReady = false;

  if (!sendRequest(timestampCheck))
    return false;

  if (!parseHeaders(true, StateManager::getTimestamp()))
  {
    m_client.stop();
    return false;
  }

  // Update sleep duration from headers
  StateManager::setSleepDuration(m_sleepDuration);

  // If keepConnectionOpen is requested, don't close - image data is ready to read
  if (keepConnectionOpen)
  {
    m_imageDataReady = true;
    Logger::log<Logger::Level::DEBUG, Logger::Topic::HTTP>("Connection kept open, image data ready\n");
    return true; // Update available, connection open
  }

  m_client.stop();
  return true; // Update available
}

bool HttpClient::startImageDownload()
{
  if (!sendRequest(false))
    return false;

  if (!parseHeaders(false, 0))
  {
    m_client.stop();
    return false;
  }

  // Client stays open for image data
  return true;
}

bool HttpClient::isConnected() { return m_client.connected() || m_client.available(); }

int HttpClient::available() { return m_client.available(); }

void HttpClient::stop()
{
  m_client.stop();
  m_imageDataReady = false;
}

uint32_t HttpClient::readBytes(uint8_t *buf, int32_t bytes)
{
  int32_t remaining = bytes;
  uint32_t startTime = millis();
  uint32_t lastDataTime = startTime;

  while ((m_client.connected() || m_client.available()) && remaining > 0)
  {
    if (m_client.available())
    {
      int16_t val = m_client.read();
      if (buf)
        *buf++ = (uint8_t)val;
      remaining--;
      lastDataTime = millis(); // Reset idle timeout on data received
    }
    else
    {
      delay(1);
    }

    uint32_t now = millis();
    // Check idle timeout (no data for too long)
    if (now - lastDataTime > IDLE_TIMEOUT_MS)
    {
      Logger::log<Logger::Level::WARNING, Logger::Topic::HTTP>("Idle timeout after {} ms without data\n",
                                                               now - lastDataTime);
      break;
    }
    // Check total timeout
    if (now - startTime > TOTAL_TIMEOUT_MS)
    {
      Logger::log<Logger::Level::WARNING, Logger::Topic::HTTP>("Total timeout after {} ms\n", now - startTime);
      break;
    }
  }

  return bytes - remaining;
}

uint8_t HttpClient::readByte()
{
  uint8_t result;
  readBytes(&result, 1);
  return result;
}

uint8_t HttpClient::readByteValid(bool *valid)
{
  uint8_t result;
  *valid = readBytes(&result, 1) == 1;
  return result;
}

uint16_t HttpClient::read16()
{
  uint16_t result;
  ((uint8_t *)&result)[0] = readByte(); // LSB
  ((uint8_t *)&result)[1] = readByte(); // MSB
  return result;
}

#ifdef USE_LTE_MODEM

// Cellular OTA. HTTPUpdate is hardwired to WiFiClient, so the download is done
// by hand here: fetch the image over the modem socket and feed it to Update.
bool HttpClient::performOTAUpdate()
{
  if (!m_otaRequired || m_otaUrl.length() == 0)
  {
    Logger::log<Logger::Level::WARNING, Logger::Topic::SYSTEM>("No OTA update URL available\n");
    return false;
  }

  stop();

  String url = m_otaUrl;

  // Follow at most one redirect (CDNs commonly bounce /latest to a versioned file)
  for (uint8_t hop = 0; hop < 2; hop++)
  {
    if (url.startsWith("https://"))
    {
      // No TLS over the modem socket - retry the same resource unencrypted.
      Logger::log<Logger::Level::WARNING, Logger::Topic::SYSTEM>(
        "OTA URL is HTTPS, retrying over plain HTTP (no TLS on the modem)\n");
      url = "http://" + url.substring(8);
    }

    if (!url.startsWith("http://"))
    {
      Logger::log<Logger::Level::ERROR, Logger::Topic::SYSTEM>("Unsupported OTA URL: {}\n", url);
      return false;
    }

    // Split "host[:port]/path"
    String rest = url.substring(7);
    int slash = rest.indexOf('/');
    String hostPart = (slash < 0) ? rest : rest.substring(0, slash);
    String path = (slash < 0) ? "/" : rest.substring(slash);

    uint16_t port = 80;
    int colon = hostPart.indexOf(':');
    if (colon > 0)
    {
      port = (uint16_t)hostPart.substring(colon + 1).toInt();
      hostPart = hostPart.substring(0, colon);
    }

    Logger::log<Logger::Level::INFO, Logger::Topic::SYSTEM>("Starting OTA update from: {}\n", url);

    if (!m_client.connect(hostPart.c_str(), port))
    {
      Logger::log<Logger::Level::ERROR, Logger::Topic::SYSTEM>("OTA: connect to {} failed\n", hostPart);
      return false;
    }

    m_client.print("GET ");
    m_client.print(path);
    m_client.print(" HTTP/1.1\r\nHost: ");
    m_client.print(hostPart);
    m_client.print("\r\nConnection: close\r\n\r\n");

    // Read the status line and headers
    int statusCode = 0;
    int32_t contentLength = -1;
    String location;

    while (m_client.connected() || m_client.available())
    {
      String line = m_client.readStringUntil('\n');
      line.trim();

      if (line.length() == 0)
        break; // end of headers

      if (statusCode == 0 && line.startsWith("HTTP/1."))
      {
        int sp = line.indexOf(' ');
        statusCode = (sp > 0) ? line.substring(sp + 1).toInt() : 0;
      }
      else if (line.startsWith("Content-Length:"))
      {
        contentLength = line.substring(15).toInt();
      }
      else if (line.startsWith("Location:"))
      {
        location = line.substring(9);
        location.trim();
      }
    }

    if ((statusCode == 301 || statusCode == 302 || statusCode == 307 || statusCode == 308) && location.length() > 0)
    {
      Logger::log<Logger::Topic::SYSTEM>("OTA: following redirect to {}\n", location);
      stop();
      url = location;
      continue;
    }

    if (statusCode != 200)
    {
      Logger::log<Logger::Level::ERROR, Logger::Topic::SYSTEM>("OTA: server returned status {}\n", statusCode);
      stop();
      return false;
    }

    if (contentLength <= 0)
    {
      // Update needs the exact size upfront; chunked responses aren't supported.
      Logger::log<Logger::Level::ERROR, Logger::Topic::SYSTEM>("OTA: missing or invalid Content-Length\n");
      stop();
      return false;
    }

    if (!Update.begin((size_t)contentLength))
    {
      Logger::log<Logger::Level::ERROR, Logger::Topic::SYSTEM>("OTA: not enough space for {} bytes ({})\n",
                                                               contentLength, Update.errorString());
      stop();
      return false;
    }

    Logger::log<Logger::Topic::SYSTEM>("OTA: downloading {} bytes...\n", contentLength);

    uint8_t buffer[512];
    int32_t written = 0;
    while (written < contentLength)
    {
      int32_t wanted = min((int32_t)sizeof(buffer), contentLength - written);
      uint32_t got = readBytes(buffer, wanted);
      if (got == 0)
        break; // readBytes already logged the timeout

      if (Update.write(buffer, got) != got)
      {
        Logger::log<Logger::Level::ERROR, Logger::Topic::SYSTEM>("OTA: flash write failed ({})\n",
                                                                 Update.errorString());
        Update.abort();
        stop();
        return false;
      }
      written += (int32_t)got;
    }

    stop();

    if (written != contentLength)
    {
      Logger::log<Logger::Level::ERROR, Logger::Topic::SYSTEM>("OTA: truncated download, {} of {} bytes\n", written,
                                                               contentLength);
      Update.abort();
      return false;
    }

    if (!Update.end(true))
    {
      Logger::log<Logger::Level::ERROR, Logger::Topic::SYSTEM>("OTA update failed: {}\n", Update.errorString());
      return false;
    }

    Logger::log<Logger::Level::INFO, Logger::Topic::SYSTEM>("OTA update successful, restarting...\n");
    LteModem::shutdown();
    delay(100);
    ESP.restart();
    return true;
  }

  Logger::log<Logger::Level::ERROR, Logger::Topic::SYSTEM>("OTA: too many redirects\n");
  return false;
}

#else // USE_LTE_MODEM

bool HttpClient::performOTAUpdate()
{
  if (!m_otaRequired || m_otaUrl.length() == 0)
  {
    Logger::log<Logger::Level::WARNING, Logger::Topic::SYSTEM>("No OTA update URL available\n");
    return false;
  }

  // Close any existing connection before OTA
  stop();

  Logger::log<Logger::Level::INFO, Logger::Topic::SYSTEM>("Starting OTA update from: {}\n", m_otaUrl);

  // Configure HTTPUpdate
  httpUpdate.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);
  httpUpdate.rebootOnUpdate(true); // Automatically restart on success

  t_httpUpdate_return result = httpUpdate.update(m_client, m_otaUrl);

  switch (result)
  {
    case HTTP_UPDATE_OK:
      Logger::log<Logger::Level::INFO, Logger::Topic::SYSTEM>("OTA update successful, restarting...\n");
      // Device will restart automatically due to rebootOnUpdate(true)
      return true;

    case HTTP_UPDATE_FAILED:
      Logger::log<Logger::Level::ERROR, Logger::Topic::SYSTEM>(
        "OTA update failed: {} ({})\n", httpUpdate.getLastErrorString(), httpUpdate.getLastError());
      return false;

    case HTTP_UPDATE_NO_UPDATES:
      Logger::log<Logger::Level::WARNING, Logger::Topic::SYSTEM>("OTA: No updates available\n");
      return false;

    default:
      Logger::log<Logger::Level::ERROR, Logger::Topic::SYSTEM>("OTA: Unknown result\n");
      return false;
  }
}

#endif // !USE_LTE_MODEM
