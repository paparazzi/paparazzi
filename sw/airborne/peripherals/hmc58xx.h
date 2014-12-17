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

/* Address and register definitions */
#include "peripherals/hmc58xx_regs.h"

struct Hmc58xxConfig {
  uint8_t rate;  ///< Data Output Rate Bits(6 -> 50Hz with HMC5843, 75Hz with HMC5883)
  uint8_t meas;  ///< Measurement configuration
  uint8_t gain;   ///< Gain configuration (1 -> +- 1 Gauss)
  uint8_t mode;   ///< Measurement mode
};

/** config status states */
enum Hmc58xxConfStatus {
  HMC_CONF_UNINIT,
  HMC_CONF_CRA,
  HMC_CONF_CRB,
  HMC_CONF_MODE,
  HMC_CONF_DONE
};

enum Hmc58xxType {
  HMC_TYPE_5843,
  HMC_TYPE_5883
};

struct Hmc58xx {
  struct i2c_periph *i2c_p;
  struct i2c_transaction i2c_trans;
  bool_t initialized;                 ///< config done flag
  enum Hmc58xxConfStatus init_status; ///< init status
  volatile bool_t data_available;     ///< data ready flag
  union {
    struct Int16Vect3 vect;           ///< data vector in mag coordinate system
    int16_t value[3];                 ///< data values accessible by channel index
  } data;
  struct Hmc58xxConfig config;
  enum Hmc58xxType type;
};


// TODO IRQ handling

// Functions
extern void hmc58xx_init(struct Hmc58xx *hmc, struct i2c_periph *i2c_p, uint8_t addr);
extern void hmc58xx_start_configure(struct Hmc58xx *hmc);
extern void hmc58xx_read(struct Hmc58xx *hmc);
extern void hmc58xx_event(struct Hmc58xx *hmc);

/// convenience function: read or start configuration if not already initialized
static inline void hmc58xx_periodic(struct Hmc58xx *hmc)
{
  if (hmc->initialized) {
    hmc58xx_read(hmc);
  } else {
    hmc58xx_start_configure(hmc);
  }
}

#endif /* HMC58XX_H */
