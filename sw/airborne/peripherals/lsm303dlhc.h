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
 * @file peripherals/lsm303dlhc.c
 *
 * Driver for Honeywell HMC5843 and HMC5883 magnetometers.
 */

#ifndef LSM303DLHC_H
#define LSM303DLHC_H

#include "std.h"
#include "mcu_periph/i2c.h"
#include "math/pprz_algebra_int.h"

/* Address and register definitions */
#include "peripherals/lsm303dlhc_regs.h"

struct Lsm303dlhcConfig {
  uint8_t rate;  ///< Data Output Rate Bits(6 -> 50Hz with HMC5843, 75Hz with HMC5883)
  uint8_t lp_mode; //Low power mode
  uint8_t scale; //full scale selection
  uint8_t hres; //high resolution output mode

  //uint8_t meas;  ///< Measurement configuration
  //uint8_t gain;   ///< Gain configuration (1 -> +- 1 Gauss)
  //uint8_t mode;   ///< Measurement mode
};

/** config status states */
enum Lsm303dlhcConfStatus {
  LSM_CONF_UNINIT,
  LSM_CONF_CTRL_REG1_A,
  LSM_CONF_CTRL_REG4_A,
  LSM_CONF_CTRL_REG3_A,
  LSM_CONF_DONE
};

struct Lsm303dlhc {
  struct i2c_periph *i2c_p;
  struct i2c_transaction i2c_trans;
  bool_t initialized;                 ///< config done flag
  enum Lsm303dlhcConfStatus init_status; ///< init status
  volatile bool_t data_available;     ///< data ready flag
  union {
    struct Int16Vect3 vect;           ///< data vector in acc coordinate system
    int16_t value[3];                 ///< data values accessible by channel index
  } data;
  struct Lsm303dlhcConfig config;
};


struct Lsm303dlhc_mag {
  struct i2c_periph *i2c_p;
  struct i2c_transaction i2c_trans;
  bool_t initialized;                 ///< config done flag
  enum Lsm303dlhcConfStatus init_status; ///< init status
  volatile bool_t data_available;     ///< data ready flag
  union {
    struct Int16Vect3 vect;           ///< data vector in mag coordinate system
    int16_t value[3];                 ///< data values accessible by channel index
  } data;
  struct Lsm303dlhcConfig config;
};


// TODO IRQ handling

// Functions
extern void lsm303dlhc_init(struct Lsm303dlhc *lsm, struct i2c_periph *i2c_p, uint8_t addr);
extern void lsm303dlhc_start_configure(struct Lsm303dlhc *lsm);
extern void lsm303dlhc_read(struct Lsm303dlhc *lsm);
extern void lsm303dlhc_event(struct Lsm303dlhc *lsm);

/// convenience function: read or start configuration if not already initialized
static inline void lsm303dlhc_periodic(struct Lsm303dlhc *lsm) {
  if (lsm->initialized)
    lsm303dlhc_read(lsm);
  else
    lsm303dlhc_start_configure(lsm);
}

#endif /* LSM303DLHC_H */
