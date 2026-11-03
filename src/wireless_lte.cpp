/*
 * LTE flavour of the Wireless module.
 *
 * Implements the exact same Wireless:: API as wireless.cpp, but backed by the
 * A7682E cellular modem instead of Wi-Fi. main.cpp therefore needs no changes.
 * There is no captive portal, so the portal helpers report "nothing to do";
 * the connection cache hooks store the last operator and band instead of a
 * BSSID, and are applied inside LteModem::begin().
 */

#include "lte_modem.h"

#ifdef USE_LTE_MODEM

  #include "logger.h"
  #include "state_manager.h"
  #include "wireless.h"

  #include <esp_system.h>
  // esp_read_mac moved to its own header in ESP-IDF 5; core 2.x still has it in
  // esp_system.h, so include the new header only when it exists.
  #if __has_include(<esp_mac.h>)
    #include <esp_mac.h>
  #endif

namespace Wireless
{

void init(const String &hostname, const String &password, void (*callback)())
{
  (void)hostname; // no AP to name
  (void)password; // no AP to protect
  (void)callback; // the callback opens the Wi-Fi config portal screen - N/A here

  Logger::log<Logger::Topic::LTE>("Bringing up cellular connection...\n");

  if (LteModem::begin())
    return;

  // No configuration portal to fall back to. Bump the failure counter here so
  // the caller's offline path still gets the usual exponential retry backoff,
  // then let main.cpp render the offline screen and go back to sleep.
  StateManager::incrementFailureCount();
  Logger::log<Logger::Level::ERROR, Logger::Topic::LTE>("Cellular connection failed\n");

  // Switch the modem off now rather than at deep sleep: the offline screen is
  // drawn next, and an e-paper refresh on top of a modem still searching (and
  // transmitting attach attempts) is exactly the combined load a weak supply
  // cannot take - besides being wasted battery.
  LteModem::shutdown();
}

void process()
{
  // No portal to service; keep URC handling alive for symmetry.
  LteModem::pumpUrc();
}

bool isConfigPortalActive() { return false; }

bool tryFastConnect()
{
  // There is no shortcut that skips bring-up entirely the way a cached BSSID
  // does for Wi-Fi - the modem still has to boot. The cached operator and band
  // are applied inside LteModem::begin() instead.
  return false;
}

void saveConnectionCache() { LteModem::saveNetworkCache(); }

void invalidateConnectionCache() { LteModem::invalidateNetworkCache(); }

String getSSID() { return LteModem::getOperator(); }

int8_t getStrength()
{
  int8_t rssi = LteModem::getRssi();
  Logger::log<Logger::Level::DEBUG, Logger::Topic::LTE>("Strength: {} dBm\n", rssi);
  return rssi;
}

String getMacAddress()
{
  // Wi-Fi radio MAC, read straight from eFuse so the device identity stays the
  // same as it would be in a Wi-Fi build (no need to start the Wi-Fi driver).
  uint8_t mac[6] = {0};
  esp_read_mac(mac, ESP_MAC_WIFI_STA);

  char buf[18];
  snprintf(buf, sizeof(buf), "%02X:%02X:%02X:%02X:%02X:%02X", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  return String(buf);
}

String getSoftAPSSID() { return String(); }

String getSoftAPIP() { return String(); }

String getIPAddress() { return LteModem::getIpAddress(); }

bool isConnected() { return LteModem::isReady(); }

void turnOff() { LteModem::shutdown(); }

void resetCredentialsAndReboot()
{
  LteModem::shutdown();

  Logger::log<Logger::Topic::LTE>("Erasing stored APN...\n");
  LteModem::clearStoredApn();

  Logger::log<Logger::Topic::SYSTEM>("Rebooting ESP...\n");
  ESP.restart();
}

} // namespace Wireless

#endif // USE_LTE_MODEM
