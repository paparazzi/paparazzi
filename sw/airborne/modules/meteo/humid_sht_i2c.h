#ifndef HUMID_SHT_I2C_H
#define HUMID_SHT_I2C_H

#include "std.h"

#define SHT2_WRITE_USER          0xE6
#define SHT2_READ_USER           0xE7
#define SHT2_TRIGGER_TEMP        0xF3
#define SHT2_TRIGGER_HUMID       0xF5
#define SHT2_SOFT_RESET          0xFE

enum sht_stat_i2c {
  SHT2_UNINIT,
  SHT2_IDLE,
  SHT2_RESET,
  SHT2_SERIAL,
  SHT2_SERIAL1,
  SHT2_SERIAL2,
  SHT2_SET_CONFIG,
  SHT2_READ_SERIAL,
  SHT2_TRIG_TEMP,
  SHT2_GET_TEMP,
  SHT2_READ_TEMP,
  SHT2_TRIG_HUMID,
  SHT2_GET_HUMID,
  SHT2_READ_HUMID
};

int8_t humid_sht_crc(volatile uint8_t *data);
void humid_sht_init_i2c(void);
void humid_sht_periodic_i2c(void);
void humid_sht_p_temp(void);
void humid_sht_p_humid(void);
void humid_sht_event_i2c(void);

extern uint16_t humidsht_i2c, tempsht_i2c;
extern float fhumidsht_i2c, ftempsht_i2c;

#endif
