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
 * @file peripherals/lis3mdl_i2c.h
 *
 * I2C interface for LIS3MDL magnetometer.
 */

#ifndef LIS3MDL_I2C_H
#define LIS3MDL_I2C_H

#include "std.h"
#include "math/pprz_algebra_int.h"
#include "mcu_periph/i2c.h"

/* Include common LIS3MDL options and definitions */
#include "peripherals/lis3mdlv2.h"

struct Lis3mdl_I2c {
  struct i2c_periph *i2c_p;
  struct i2c_transaction i2c_trans;
  enum Lis3mdlConfStatus init_status; ///< init status
  bool initialized;                   ///< config done flag
  volatile bool data_available;       ///< data ready flag
  union {
    struct Int16Vect3 vect;           ///< data vector
    int16_t value[3];                 ///< data values accessible by channel index
  } data;
  struct Lis3mdlConfig config;
};

// Functions
extern void lis3mdl_i2c_init(struct Lis3mdl_I2c *lis, struct i2c_periph *i2c_p, uint8_t addr);
extern void lis3mdl_i2c_start_configure(struct Lis3mdl_I2c *lis);
extern void lis3mdl_i2c_read(struct Lis3mdl_I2c *lis);
extern void lis3mdl_i2c_event(struct Lis3mdl_I2c *lis);

// convenience function: read or start configuration if not already initialized
static inline void lis3mdl_i2c_periodic(struct Lis3mdl_I2c *lis)
{
  if (lis->initialized) {
    lis3mdl_i2c_read(lis);
  } else {
    lis3mdl_i2c_start_configure(lis);
  }
}

#endif // LIS3MDL_I2C_H
