/*
 * Copyright (C) 2011 Gautier Hattenbergerr <gautier.hattenberger@enac.fr>
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
 */

/**
 * @file peripherals/hmc58xx.c
 *
 * Driver for Honeywell HMC5843 and HMC5883 magnetometers.
 */

#ifndef HMC58XX_H
#define HMC58XX_H

#include "std.h"
#include "mcu_periph/i2c.h"
#include "math/pprz_algebra_int.h"

struct Hmc58xxConfig {
  uint8_t dor;  ///< Data Output Rate Bits(6 -> 50Hz with HMC5843, 75Hz with HMC5883)
  uint8_t ms;   ///< Measurement configuration
  uint8_t gn;   ///< Gain configuration (1 -> +- 1 Gauss)
  uint8_t md;   ///< Measurement mode
};

struct Hmc58xx {
  struct i2c_periph *i2c_dev_p;
  struct i2c_transaction i2c_trans;
  bool_t initialized;               ///< config done flag
  bool_t init_status;               ///< init status
  volatile bool_t data_available;   ///< data ready flag
  union {
    struct Int16Vect3 vect;         ///< data vector in mag coordinate system
    int16_t value[3];               ///< data values accessible by channel index
  } data;
  struct Hmc58xxConfig config;
};


// TODO IRQ handling

// Functions
extern void hmc58xx_init(struct Hmc58xx *hmc, bool_t use_default_config);
extern void hmc58xx_configure(struct Hmc58xx *hmc);
extern void hmc58xx_read(struct Hmc58xx *hmc);
extern void hmc58xx_event(struct Hmc58xx *hmc);

// Macro for using HMC58XX in periodic function
#define Hmc58xxPeriodic(_hmc) {                 \
  if (_hmc.initialized) hmc58xx_read(&_hmc);  \
  else hmc58xx_configure(&_hmc);                 \
}

#if 0 // doesn't seem to be needed/used anywhere?
#define MagEvent(_m_handler) {    \
    hmc58xx_event();              \
    if (hmc->data_available) { \
      _m_handler();               \
    }                             \
  }
#endif

#endif /* HMC58XX_H */
