#ifndef SENSOR_I2C_H
#define SENSOR_I2C_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "stdbool.h"


#define SENSOR_I2C_0     0
#define SENSOR_I2C_1     1
#define SENSOR_I2C_NONE  -1


bool SensorI2C_open(void);
bool SensorI2C_select(uint8_t interface, uint8_t slaveAddress);
bool SensorI2C_readReg(uint8_t addr, uint8_t *pBuf, uint8_t nBytes);
bool SensorI2C_writeReg(uint8_t addr, uint8_t *pBuf, uint8_t nBytes);
void SensorI2C_deselect(void);
void SensorI2C_close(void);

bool SensorI2C_read(uint8_t *data, uint8_t len);
bool SensorI2C_write(uint8_t *data, uint8_t len);

////////////////////////////////////////////////////////////////////////////////
#ifdef __cplusplus
}
#endif

#endif /* SENSOR_I2C_H */
