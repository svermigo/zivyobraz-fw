#include "lte_client.h"

#ifdef USE_LTE_MODEM

  #include "logger.h"

namespace
{
// Only one socket exists on the modem, so the buffers are shared module state
// rather than per-instance members - keeps HttpClient off the task stack.
uint8_t g_txBuffer[LTE_IO_CHUNK];
size_t g_txLen = 0;

uint8_t g_rxBuffer[LTE_IO_CHUNK];
size_t g_rxLen = 0;
size_t g_rxPos = 0;

// Minimum spacing between AT+CIPRXGET polls that came back empty, so a caller
// spinning on read() doesn't flood the modem with AT traffic.
constexpr uint32_t EMPTY_POLL_INTERVAL_MS = 20;
} // namespace

LteClient::LteClient() : m_connected(false), m_lastEmptyPoll(0), m_finalDrainDone(false)
{
  // Header lines are read with readStringUntil(), which relies on the Stream
  // timeout; the default 1 s is too tight for a mobile network round trip.
  setTimeout(15000);
}

LteClient::~LteClient() { stop(); }

int LteClient::connect(IPAddress ip, uint16_t port) { return connect(ip.toString().c_str(), port); }

int LteClient::connect(const char *host, uint16_t port)
{
  stop();

  g_txLen = 0;
  g_rxLen = 0;
  g_rxPos = 0;

  if (!LteModem::isReady())
  {
    Logger::log<Logger::Level::ERROR, Logger::Topic::LTE>("Modem not ready, cannot connect\n");
    return 0;
  }

  if (!LteModem::socketOpen(host, port))
    return 0;

  m_connected = true;
  m_finalDrainDone = false;
  return 1;
}

size_t LteClient::write(uint8_t b) { return write(&b, 1); }

size_t LteClient::write(const uint8_t *buf, size_t size)
{
  if (!m_connected)
    return 0;

  size_t written = 0;
  while (written < size)
  {
    size_t space = sizeof(g_txBuffer) - g_txLen;
    size_t chunk = min(space, size - written);
    memcpy(g_txBuffer + g_txLen, buf + written, chunk);
    g_txLen += chunk;
    written += chunk;

    // A failed flush discards the buffer, this call's last chunk included
    if (g_txLen == sizeof(g_txBuffer) && !flushTx())
      return written - chunk;
  }

  return written;
}

bool LteClient::flushTx()
{
  if (g_txLen == 0)
    return true;

  size_t pending = g_txLen;
  g_txLen = 0;

  int sent = LteModem::socketSend(g_txBuffer, pending);
  if (sent < 0 || (size_t)sent != pending)
  {
    Logger::log<Logger::Level::ERROR, Logger::Topic::LTE>("Send failed ({} of {} bytes)\n", sent, pending);
    m_connected = false;
    return false;
  }

  return true;
}

bool LteClient::fillRx()
{
  if (g_rxPos < g_rxLen)
    return true;

  g_rxPos = 0;
  g_rxLen = 0;

  if (!LteModem::socketIsOpen())
    return false;

  // Back off after an empty poll so tight read() loops stay cheap
  if (millis() - m_lastEmptyPoll < EMPTY_POLL_INTERVAL_MS)
    return false;

  int rest = 0;
  int got = LteModem::socketRecv(g_rxBuffer, sizeof(g_rxBuffer), &rest);

  if (got < 0)
  {
    m_connected = false;
    return false;
  }

  if (got == 0)
  {
    m_lastEmptyPoll = millis();
    return false;
  }

  g_rxLen = (size_t)got;
  return true;
}

int LteClient::available()
{
  flushTx();

  if (g_rxPos < g_rxLen)
    return (int)(g_rxLen - g_rxPos);

  fillRx();
  return (int)(g_rxLen - g_rxPos);
}

int LteClient::read()
{
  if (available() == 0)
    return -1;
  return g_rxBuffer[g_rxPos++];
}

int LteClient::read(uint8_t *buf, size_t size)
{
  int total = 0;
  while (size > 0)
  {
    if (available() == 0)
      break;

    size_t chunk = min(size, g_rxLen - g_rxPos);
    memcpy(buf, g_rxBuffer + g_rxPos, chunk);
    g_rxPos += chunk;
    buf += chunk;
    size -= chunk;
    total += (int)chunk;
  }
  // Like WiFiClient: 0 means "nothing yet", -1 only once the stream is over
  if (total == 0 && !connected())
    return -1;
  return total;
}

int LteClient::peek()
{
  if (available() == 0)
    return -1;
  return g_rxBuffer[g_rxPos];
}

void LteClient::flush() { flushTx(); }

void LteClient::stop()
{
  g_txLen = 0;
  g_rxLen = 0;
  g_rxPos = 0;

  if (m_connected || LteModem::socketIsOpen())
    LteModem::socketClose();

  m_connected = false;
}

uint8_t LteClient::connected()
{
  // Buffered payload still counts as connected, mirroring WiFiClient - the peer
  // may have closed while data is still queued on the modem.
  if (g_rxPos < g_rxLen)
    return 1;

  if (!m_connected)
    return 0;

  if (LteModem::socketRemoteClosed() && !LteModem::socketHasPendingData())
  {
    // The peer has closed, but the modem can surface +IPCLOSE before it has
    // handed us the tail of the stream. Poll once more - and again after every
    // chunk that still arrives - instead of dropping the last bytes of an image.
    if (!m_finalDrainDone)
    {
      m_finalDrainDone = true;
      m_lastEmptyPoll = 0; // bypass the empty-poll backoff for this one
      if (fillRx())
      {
        m_finalDrainDone = false;
        return 1;
      }
    }

    m_connected = false;
    return 0;
  }

  return 1;
}

#endif // USE_LTE_MODEM
