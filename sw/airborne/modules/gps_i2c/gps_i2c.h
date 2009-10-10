#ifndef GPS_I2C
#define GPS_I2C

#include <inttypes.h>

#define GPS_I2C_BUF_SIZE 256
extern uint8_t gps_i2c_buf[GPS_I2C_BUF_SIZE];
extern uint8_t gps_i2c_insert_idx, gps_i2c_extract_idx;

void gps_i2c_init(void);
void gps_i2c_event(void);
void gps_i2c_periodic(void);

#define gps_i2c_AddCharToBuf(_x) { \
  gps_i2c_buf[gps_i2c_insert_idx] = _x; \
  gps_i2c_insert_idx++; /* size=256, No check for buf overflow */ \
}

#endif // GPS_I2C
