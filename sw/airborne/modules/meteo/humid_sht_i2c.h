#ifndef HUMID_SHT_I2C_H
#define HUMID_SHT_I2C_H

#include "std.h"

#define SHT_WRITE_USER          0xE6
#define SHT_READ_USER           0xE7
#define SHT_TRIGGER_TEMP        0xF3
#define SHT_TRIGGER_HUMID       0xF5
#define SHT_SOFT_RESET          0xFE

enum sht_stat{
  SHT_UNINIT,
  SHT_IDLE,
  SHT_RESET,
  SHT_SERIAL,
  SHT_SERIAL1,
  SHT_SERIAL2,
  SHT_SET_CONFIG,
  SHT_READ_SERIAL,
  SHT_TRIG_TEMP,
  SHT_GET_TEMP,
  SHT_READ_TEMP,
  SHT_TRIG_HUMID,
  SHT_GET_HUMID,
  SHT_READ_HUMID
};

int8_t humid_sht_crc(volatile uint8_t* data);
void humid_sht_init(void);
void humid_sht_periodic(void);
void humid_sht_p_temp(void);
void humid_sht_p_humid(void);
void humid_sht_event(void);

extern uint16_t humidsht, tempsht;
extern float fhumidsht, ftempsht;

#endif
