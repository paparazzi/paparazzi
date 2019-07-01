/*
 * Copyright (C) 2019 Alexis Cornard <alexiscornard@gmail.com>
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
 * @file peripherals/lsm6ds33_i2c.h
 *
 * Driver for the accelerometer and gyrometer LSM6DS33.
 */

#ifndef LSM6_I2C_H
#define LSM6_I2C_H

#include "std.h"
#include "math/pprz_algebra_int.h"
#include "mcu_periph/i2c.h"

/* Include common LSM6DS33 options and definitions */
#include "peripherals/lsm6ds33.h"

struct Lsm6_I2c {
  struct i2c_periph *i2c_p;
  struct i2c_transaction i2c_trans;
  enum Lsm6ConfStatus init_status;  ///< init status
  bool initialized;                 ///< config done flag
  volatile bool data_available;     ///< data ready flag
  union {
    struct Int16Vect3 vect;         ///< data vector in accel coordinate system
    int16_t value[3];               ///< data values accessible by channel index
  } data_xl;
  union {
    struct Int16Rates rates;        ///< data as angular rates in gyroscop coordinate system
    int16_t value[3];               ///< data values accessible by channel index
  } data_g;
  struct Lsm6Config config;
};

// Functions
extern void lsm6_i2c_init(struct Lsm6_I2c *lsm, struct i2c_periph *i2c_p, uint8_t addr);
extern void lsm6_i2c_start_configure(struct Lsm6_I2c *lsm);
extern void lsm6_i2c_read(struct Lsm6_I2c *lsm);
extern void lsm6_i2c_event(struct Lsm6_I2c *lsm);

/// convenience function: read or start configuration if not already initialized
static inline void lsm6_i2c_periodic(struct Lsm6_I2c *lsm)
{
  if (lsm->initialized) {
    lsm6_i2c_read(lsm);
  } else {
    lsm6_i2c_start_configure(lsm);
  }
}

#endif // LSM6_I2C_H
