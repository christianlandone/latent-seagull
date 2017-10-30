#ifndef SENSOR_UTIL_H
#define SENSOR_UTIL_H

#ifdef __cplusplus
extern "C"
{
#endif

#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/knl/Task.h>

/* Boolean assertion; return false Failed) and release I2C if condition is not met */
#define ST_ASSERT(cond)   ST( if (!(cond)) {SensorI2C_deselect(); return false;})

/* Void assertion; return and release I2C if condition is not met */
#define ST_ASSERT_V(cond) ST( if (!(cond)) {SensorI2C_deselect(); return;} )

/* Loop enclosure for macros */
#define ST(x)             do { x } while (__LINE__ == -1)

/* Conversion macros */
#define HI_UINT16(a)     (((a) >> 8) & 0xFF)
#define LO_UINT16(a)     ((a) & 0xFF)
#define SWAP(v)          ((LO_UINT16(v) << 8) | HI_UINT16(v))

/* Delay */
#define DELAY_MS(i)      (Task_sleep(((i) * 1000) / Clock_tickPeriod))
#define DELAY_US(i)      (Task_sleep(((i) * 1) / Clock_tickPeriod))
#define MS_2_TICKS(ms)   (((ms) * 1000) / Clock_tickPeriod)

void     SensorUtil_convertToLe(uint8_t *data, uint8_t len);
uint16_t SensorUtil_floatToSfloat(float data);
float    SensorUtil_sfloatToFloat(uint16_t rawData);
uint16_t SensorUtil_intToSfloat(int data);


#ifdef __cplusplus
}
#endif

#endif /* SENSOR_UTIL_H */
