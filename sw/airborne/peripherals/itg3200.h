/*
 * Copyright (C) 2011 Gautier Hattenberger <gautier.hattenberger@enac.fr>
 *               2013 Felix Ruess <felix.ruess@gmail.com>
 *
 * This file is part of paparazzi.
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 *
 */

/**
 * @file peripherals/itg3200.h
 *
 * Driver for the gyro ITG3200 from InvenSense.
 */

#ifndef ITG3200_H
#define ITG3200_H

#include "std.h"
#include "math/pprz_algebra_int.h"
#include "mcu_periph/i2c.h"

/* Address and register definitions */
#include "peripherals/itg3200_regs.h"

/// Default sample rate divider
#define ITG3200_DEFAULT_SMPLRT_DIV 0
/// Default full scale range +- 2000°/s
#define ITG3200_DEFAULT_FS_SEL 3
/// Default internal sampling (1kHz, 42Hz LP Bandwidth)
#define ITG3200_DEFAULT_DLPF_CFG ITG3200_DLPF_42HZ
/// Default interrupt config: RAW_RDY_EN
#define ITG3200_DEFAULT_INT_CFG 1
/// Default clock: PLL with X gyro reference
#define ITG3200_DEFAULT_CLK_SEL 1


struct Itg3200Config {
  uint8_t smplrt_div; ///< Sample rate divider
  uint8_t fs_sel;     ///< Full scale range
  enum Itg3200DLPF dlpf_cfg;  ///< Digital Low Pass Filter
  uint8_t int_cfg;    ///< Interrupt config
  uint8_t clk_sel;    ///< Clock select
};

/** config status states */
enum Itg3200ConfStatus {
  ITG_CONF_UNINIT,
  ITG_CONF_SD,
  ITG_CONF_DF,
  ITG_CONF_INT,
  ITG_CONF_PWR,
  ITG_CONF_DONE
};

struct Itg3200 {
  struct i2c_periph *i2c_p;
  struct i2c_transaction i2c_trans;
  bool initialized;                 ///< config done flag
  enum Itg3200ConfStatus init_status; ///< init status
  volatile bool data_available;     ///< data ready flag
  union {
    struct Int32Rates rates;          ///< data as angular rates in gyro coordinate system
    int32_t value[3];                 ///< data values accessible by channel index
  } data;
  struct Itg3200Config config;
};

// TODO IRQ handling

// Functions
extern void itg3200_init(struct Itg3200 *itg, struct i2c_periph *i2c_p, uint8_t i2c_address);
extern void itg3200_set_default_config(struct Itg3200Config *conf);
extern void itg3200_start_configure(struct Itg3200 *itg);
extern void itg3200_read(struct Itg3200 *itg);
extern void itg3200_event(struct Itg3200 *itg);

/// convenience function: read or start configuration if not already initialized
static inline void itg3200_periodic(struct Itg3200 *itg)
{
  if (itg->initialized) {
    itg3200_read(itg);
  } else {
    itg3200_start_configure(itg);
  }
}

#endif // ITG3200_H
