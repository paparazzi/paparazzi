/*
 * Copyright (C) 2019 Gautier Hattenberger <gautier.hattenberger@enac.fr>
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
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */

/**
 * @file peripherals/bmi088_i2c.h
 *
 * Driver for the BMI088 using I2C.
 */

#ifndef BMI088_I2C_H
#define BMI088_I2C_H

#include "std.h"
#include "math/pprz_algebra_int.h"
#include "mcu_periph/i2c.h"

/* Include common BMI088 options and definitions */
#include "peripherals/bmi088.h"

struct Bmi088_I2c {
  struct i2c_periph *i2c_p;
  struct i2c_transaction gyro_trans;  ///< i2c transaction for gyro
  struct i2c_transaction accel_trans; ///< i2c transaction for accel
  volatile bool gyro_available;       ///< gyro data ready flag
  volatile bool accel_available;      ///< accel data ready flag
  union {
    struct Int16Vect3 vect;           ///< accel data vector in accel coordinate system
    int16_t value[3];                 ///< accel data values accessible by channel index
  } data_accel;
  union {
    struct Int16Rates rates;          ///< rates data as angular rates in gyro coordinate system
    int16_t value[3];                 ///< rates data values accessible by channel index
  } data_rates;
  struct Bmi088Config config;
};

// Functions
extern void bmi088_i2c_init(struct Bmi088_I2c *bmi, struct i2c_periph *i2c_p, uint8_t gyro_addr, uint8_t accel_addr);
extern void bmi088_i2c_start_configure(struct Bmi088_I2c *bmi);
extern void bmi088_i2c_read(struct Bmi088_I2c *bmi);
extern void bmi088_i2c_event(struct Bmi088_I2c *bmi);

/// convenience function: read or start configuration if not already initialized
static inline void bmi088_i2c_periodic(struct Bmi088_I2c *bmi)
{
  if (bmi->config.initialized) {
    bmi088_i2c_read(bmi);
  } else {
    bmi088_i2c_start_configure(bmi);
  }
}

#endif // BMI088_I2C_H
