#ifndef HUMID_SHT_I2C_H
#define HUMID_SHT_I2C_H

#include "mcu_periph/i2c.h"
#include "modules/core/threads.h"
#include "std.h"

enum sht_stat_i2c {
  SHT2_UNINIT,
  SHT2_READING,
};


struct sht_humid_t {
  struct i2c_transaction sht_trans;
  enum sht_stat_i2c sht_status;
  uint32_t sht_serial1;
  uint32_t sht_serial2;
  uint16_t humidsht_i2c;
  uint16_t tempsht_i2c;
  float fhumidsht_i2c;
  float ftempsht_i2c;

  pprz_thread_t thd_handle;
  pprz_bsem_t bsem_sht_status;
};


void humid_sht_init_i2c(void);
void humid_sht_periodic_i2c(void);

#endif
