#include "epdiy_i2c_bridge.h"

static bool g_epdiy_i2c_initialized = false;

extern "C" bool epdiy_i2c_is_initialized(void)
{
  return g_epdiy_i2c_initialized;
}

extern "C" void epdiy_set_i2c_initialized(bool initialized)
{
  g_epdiy_i2c_initialized = initialized;
}
