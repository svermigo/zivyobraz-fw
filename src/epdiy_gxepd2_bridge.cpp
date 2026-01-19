#include "display.h"
#include "epdiy_gxepd2_bridge.h"

#if DISPLAY_ID == DT_ED097TC2_EPDIY
  #define EPDIY_DISPLAY ED097TC2
  #define EPDIY_VCOM_VOLTAGE 1500
#elif DISPLAY_ID == DT_ED060XC3_EPDIY
  #define EPDIY_DISPLAY ED060XC3
  #define EPDIY_VCOM_VOLTAGE 100
#else
  #error "Unknown epdiy display type"
#endif

static uint8_t colorToEpdiy(uint16_t color)
{
  if (color == GxEPD_WHITE) return 0xFF;
  if (color == GxEPD_BLACK) return 0x00;
  if (color == GxEPD_LIGHTGREY) return 0xAA;
  if (color == GxEPD_DARKGREY) return 0x55;

  uint8_t r = (color >> 11) & 0x1F;
  uint8_t g = (color >> 5) & 0x3F;
  uint8_t b = color & 0x1F;
  uint8_t gray = ((r * 8 + g * 4 + b * 8) / 3) & 0xF0;
  return gray | (gray >> 4);
}

EpdiyDisplay::EpdiyEpd2::EpdiyEpd2(EpdiyDisplay *owner)
  : WIDTH(0), HEIGHT(0), hasPartialUpdate(false), owner_(owner)
{
}

void EpdiyDisplay::EpdiyEpd2::selectSPI(SPIClass &spi, SPISettings settings)
{
  (void)spi;
  (void)settings;
}

void EpdiyDisplay::EpdiyEpd2::setPaged()
{
}

void EpdiyDisplay::EpdiyEpd2::setBusyCallback(void (*callback)(const void *), void *context)
{
  (void)callback;
  (void)context;
}

void EpdiyDisplay::EpdiyEpd2::refresh(bool partial)
{
  owner_->refreshDisplay(partial);
}

void EpdiyDisplay::EpdiyEpd2::writeImage(const uint8_t *black, const uint8_t *color, int16_t x, int16_t y, int16_t w,
                                         int16_t h, bool invert, bool mirror, bool pgm)
{
  (void)color;
  (void)invert;
  (void)mirror;
  (void)pgm;

  owner_->ensureInit();
  if (!black) return;
  if (!owner_->framebuffer_) return;

  for (int16_t row = 0; row < h; row++)
  {
    for (int16_t col = 0; col < w; col++)
    {
      uint16_t idx = (row * ((w + 7) / 8)) + (col / 8);
      uint8_t bit = 7 - (col % 8);
      bool is_black = (black[idx] >> bit) & 0x01;
      epd_draw_pixel(x + col, y + row, is_black ? 0x00 : 0xFF, owner_->framebuffer_);
    }
  }
}

void EpdiyDisplay::EpdiyEpd2::writeImage(const uint8_t *black, int16_t x, int16_t y, int16_t w, int16_t h, bool invert,
                                         bool mirror, bool pgm)
{
  writeImage(black, nullptr, x, y, w, h, invert, mirror, pgm);
}

void EpdiyDisplay::EpdiyEpd2::writeImage_4G(const uint8_t *data, uint8_t level, int16_t x, int16_t y, int16_t w,
                                            int16_t h, bool invert, bool mirror, bool pgm)
{
  (void)level;
  (void)invert;
  (void)mirror;
  (void)pgm;

  owner_->ensureInit();
  if (!data) return;
  if (!owner_->framebuffer_) return;

  for (int16_t row = 0; row < h; row++)
  {
    for (int16_t col = 0; col < w; col++)
    {
      uint16_t idx = (row * ((w + 3) / 4)) + (col / 4);
      uint8_t bit_pos = (3 - (col % 4)) * 2;
      uint8_t pixel_val = (data[idx] >> bit_pos) & 0x03;

      uint8_t epd_color;
      switch (pixel_val)
      {
        case 0: epd_color = 0x00; break;
        case 1: epd_color = 0x55; break;
        case 2: epd_color = 0xAA; break;
        case 3: epd_color = 0xFF; break;
        default: epd_color = 0xFF; break;
      }

      epd_draw_pixel(x + col, y + row, epd_color, owner_->framebuffer_);
    }
  }
}

void EpdiyDisplay::EpdiyEpd2::writeNative(const uint8_t *data, const uint8_t *color, int16_t x, int16_t y, int16_t w,
                                          int16_t h, bool invert, bool mirror, bool pgm)
{
  (void)data;
  (void)color;
  (void)x;
  (void)y;
  (void)w;
  (void)h;
  (void)invert;
  (void)mirror;
  (void)pgm;
}

EpdiyDisplay::EpdiyDisplay()
  : Adafruit_GFX(0, 0), epd2(this), framebuffer_(nullptr), initialized_(false), page_active_(false)
{
}

void EpdiyDisplay::init()
{
  if (initialized_) return;

  epd_init(&sverio_paperboard_v1, &EPDIY_DISPLAY, EPD_LUT_64K);
  epd_set_vcom(EPDIY_VCOM_VOLTAGE);
  hl_ = epd_hl_init(EPD_BUILTIN_WAVEFORM);
  epd_set_rotation(EPD_ROT_LANDSCAPE);
  framebuffer_ = epd_hl_get_framebuffer(&hl_);
  initialized_ = true;
  updateDimensions();
}

void EpdiyDisplay::init(uint32_t baud)
{
  (void)baud;
  init();
}

void EpdiyDisplay::init(uint32_t baud, bool initial, uint16_t reset, bool pulldown)
{
  (void)baud;
  (void)initial;
  (void)reset;
  (void)pulldown;
  init();
}

void EpdiyDisplay::powerOff()
{
  epd_poweroff();
}

void EpdiyDisplay::setRotation(uint8_t rotation)
{
  EpdRotation rot = EPD_ROT_LANDSCAPE;
  switch (rotation)
  {
    case 0: rot = EPD_ROT_LANDSCAPE; break;
    case 1: rot = EPD_ROT_PORTRAIT; break;
    case 2: rot = EPD_ROT_INVERTED_LANDSCAPE; break;
    case 3: rot = EPD_ROT_INVERTED_PORTRAIT; break;
  }
  epd_set_rotation(rot);
  updateDimensions();
}

void EpdiyDisplay::setFullWindow()
{
}

void EpdiyDisplay::setPartialWindow(int16_t x, int16_t y, int16_t w, int16_t h)
{
  (void)x;
  (void)y;
  (void)w;
  (void)h;
}

void EpdiyDisplay::fillScreen(uint16_t color)
{
  fillRect(0, 0, width(), height(), color);
}

void EpdiyDisplay::fillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color)
{
  ensureInit();
  if (!framebuffer_) return;

  EpdRect rect = {x, y, w, h};
  epd_fill_rect(rect, colorToEpdiy(color), framebuffer_);
}

void EpdiyDisplay::drawPixel(int16_t x, int16_t y, uint16_t color)
{
  ensureInit();
  if (!framebuffer_) return;

  epd_draw_pixel(x, y, colorToEpdiy(color), framebuffer_);
}

void EpdiyDisplay::drawPixel8bit(int16_t x, int16_t y, uint8_t gray)
{
  ensureInit();
  if (!framebuffer_) return;

  epd_draw_pixel(x, y, gray, framebuffer_);
}

void EpdiyDisplay::firstPage()
{
  ensureInit();
  page_active_ = true;

  if (framebuffer_)
  {
    memset(framebuffer_, 0xFF, epd_width() * epd_height() / 2);
  }
}

bool EpdiyDisplay::nextPage()
{
  if (!page_active_) return false;
  refreshDisplay(false);
  page_active_ = false;
  return false;
}

uint16_t EpdiyDisplay::pages() const
{
  return 1;
}

void EpdiyDisplay::ensureInit()
{
  if (!initialized_) init();
}

void EpdiyDisplay::updateDimensions()
{
  epd2.WIDTH = epd_width();
  epd2.HEIGHT = epd_height();
  _width = epd_rotated_display_width();
  _height = epd_rotated_display_height();
}

void EpdiyDisplay::refreshDisplay(bool partial)
{
  (void)partial;
  ensureInit();

  epd_poweron();
  epd_clear();
  epd_hl_update_screen(&hl_, MODE_EPDIY_WHITE_TO_GL16, epd_ambient_temperature());
  epd_poweroff();
}
