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
 * @file peripherals/adxl345_i2c.h
 *
 * Driver for the accelerometer ADXL345 from Analog Devices using I2C.
 */

#ifndef ADXL345_I2C_H
#define ADXL345_I2C_H

#include "std.h"
#include "math/pprz_algebra_int.h"
#include "mcu_periph/i2c.h"

/* Include common ADXL345 options and definitions */
#include "peripherals/adxl345.h"

struct Adxl345_I2c {
  struct i2c_periph *i2c_p;
  struct i2c_transaction i2c_trans;
  enum Adxl345ConfStatus init_status; ///< init status
  bool_t initialized;                 ///< config done flag
  volatile bool_t data_available;     ///< data ready flag
  union {
    struct Int16Vect3 vect;           ///< data vector in accel coordinate system
    int16_t value[3];                 ///< data values accessible by channel index
  } data;
  struct Adxl345Config config;
};

// Functions
extern void adxl345_i2c_init(struct Adxl345_I2c *adxl, struct i2c_periph *i2c_p, uint8_t addr);
extern void adxl345_i2c_start_configure(struct Adxl345_I2c *adxl);
extern void adxl345_i2c_read(struct Adxl345_I2c *adxl);
extern void adxl345_i2c_event(struct Adxl345_I2c *adxl);

/// convenience function: read or start configuration if not already initialized
static inline void adxl345_i2c_periodic(struct Adxl345_I2c *adxl)
{
  if (adxl->initialized) {
    adxl345_i2c_read(adxl);
  } else {
    adxl345_i2c_start_configure(adxl);
  }
}

#endif // ADXL345_I2C_H
