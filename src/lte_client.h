#ifndef LTE_CLIENT_H
#define LTE_CLIENT_H

#ifdef USE_LTE_MODEM

  #include "lte_modem.h"

  #include <Arduino.h>
  #include <Client.h>

/**
 * Arduino Client implementation on top of the A7682E socket AT commands.
 *
 * Deliberately API-compatible with WiFiClient so HttpClient (and anything else
 * speaking the Client/Stream interface) works unchanged. Only a single socket
 * exists on the modem, so only one instance may be connected at a time.
 *
 * Writes are buffered and flushed with AT+CIPSEND, reads are pulled in chunks
 * with AT+CIPRXGET - the modem never pushes payload at us on its own, which
 * keeps the flow-control-less UART link from overrunning.
 */
class LteClient : public Client
{
public:
  LteClient();
  ~LteClient() override;

  int connect(IPAddress ip, uint16_t port) override;
  int connect(const char *host, uint16_t port) override;

  size_t write(uint8_t b) override;
  size_t write(const uint8_t *buf, size_t size) override;

  int available() override;
  int read() override;
  int read(uint8_t *buf, size_t size) override;
  int peek() override;
  void flush() override;
  void stop() override;
  uint8_t connected() override;

  operator bool() override { return connected(); }

private:
  bool flushTx();
  bool fillRx();

  bool m_connected;
  uint32_t m_lastEmptyPoll;
  bool m_finalDrainDone;
};

#endif // USE_LTE_MODEM
#endif // LTE_CLIENT_H
