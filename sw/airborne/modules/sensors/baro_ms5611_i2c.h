#ifndef BARO_MS56111_I2C_H
#define BARO_MS56111_I2C_H

#include "std.h"
#include "peripherals/ms5611_i2c.h"

/// new measurement with every baro_ms5611_read() call
#define BARO_MS5611_DT BARO_MS5611_READ_PERIOD
#define BARO_MS5611_R 20
#define BARO_MS5611_SIGMA2 1
extern float baro_ms5611_r;
extern float baro_ms5611_sigma2;

extern float baro_ms5611_alt;
extern bool_t baro_ms5611_alt_valid;
extern bool_t baro_ms5611_enabled;

extern struct Ms5611_I2c baro_ms5611;

extern void baro_ms5611_init(void);
extern void baro_ms5611_read(void);
extern void baro_ms5611_periodic_check(void);
extern void baro_ms5611_event(void);
extern void baro_ms5611_send_coeff(void);

#define BaroMs5611UpdatePressure(_b, _h) { if (baro_ms5611.data_available) { _b = baro_ms5611.data.pressure; _h(); baro_ms5611.data_available = FALSE; } }

#define BaroMs5611UpdateAlt(_b, _h) { if (baro_ms5611.data_available) { _b = baro_ms5611_alt; _h(); baro_ms5611.data_available = FALSE; } }

#endif
