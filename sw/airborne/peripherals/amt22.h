#pragma once

#include "mcu_periph/spi.h"
#include "modules/core/threads.h"

enum amt22_type {
  AMT22_12_SINGLE,  ///< 12-bits, single-turn
  AMT22_14_SINGLE,  ///< 14-bits, single-turn
  AMT22_12_MULTI,   ///< 12-bits, multi-turn
  AMT22_14_MULTI,   ///< 14-bits, multi-turn
};

typedef struct {
  struct spi_periph *p;
  uint8_t slave_idx;
  enum amt22_type type;
} amt22_config_t;

typedef struct {
  amt22_config_t *config;

  // private
  struct spi_transaction trans;
  volatile uint8_t spi_input_buf[4];
  volatile uint8_t spi_output_buf[4];

  uint16_t position;
  int16_t turns;

  pprz_thread_t thd_handle;
  pprz_bsem_t bsem_amt22_read;
  pprz_mutex_t mtx;
} amt22_t;

void amt22_init(amt22_t* amt22, amt22_config_t* conf);
void amt22_periodic(amt22_t* amt22);
uint16_t amt22_get_position(amt22_t* amt22);
int16_t amt22_get_turns(amt22_t* amt22);
