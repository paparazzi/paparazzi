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
 * @file peripherals/lsm303dlhc.h
 *
 * Driver for ST LSM303DLHC 3D accelerometer and magnetometer.
 * UNTESTED
 */

#ifndef LSM303DLHC_H
#define LSM303DLHC_H

#include "std.h"
#include "mcu_periph/i2c.h"
#include "math/pprz_algebra_int.h"

#include "peripherals/lsm303dlhc.h"

#warning "The Lsm303dlhc I2C driver has not been tested. Use with caution."

struct Lsm303dlhc_i2c {
  struct i2c_periph *i2c_p;
  struct i2c_transaction i2c_trans;
  bool initialized;                 ///< config done flag
  union {
    enum Lsm303dlhcAccConfStatus acc; ///< init status
    enum Lsm303dlhcMagConfStatus mag; ///< init status
  } init_status;
  volatile bool data_available;     ///< data ready flag
  union {
    struct Int16Vect3 vect;           ///< data vector in acc coordinate system
    int16_t value[3];                 ///< data values accessible by channel index
  } data_accel;
  union {
    struct Lsm303dlhcAccConfig acc;
    struct Lsm303dlhcMagConfig mag;
  } config;
};

// TODO IRQ handling

// Functions
extern void lsm303dlhc_i2c_init(struct Lsm303dlhc_i2c *lsm, struct i2c_periph *i2c_p, uint8_t addr);
extern void lsm303dlhc_i2c_start_configure(struct Lsm303dlhc_i2c *lsm);
extern void lsm303dlhc_i2c_read(struct Lsm303dlhc_i2c *lsm);
extern void lsm303dlhc_i2c_event(struct Lsm303dlhc_i2c *lsm);

/// convenience function: read or start configuration if not already initialized
static inline void lsm303dlhc_i2c_periodic(struct Lsm303dlhc_i2c *lsm)
{
  if (lsm->initialized) {
    lsm303dlhc_i2c_read(lsm);
  } else {
    lsm303dlhc_i2c_start_configure(lsm);
  }
}

#endif /* LSM303DLHC_H */
