#ifndef EPDIY_I2C_BRIDGE_H
#define EPDIY_I2C_BRIDGE_H

#ifdef __cplusplus
extern "C" {
#endif

bool epdiy_i2c_is_initialized(void);
void epdiy_set_i2c_initialized(bool initialized);

#ifdef __cplusplus
}
#endif

#endif
