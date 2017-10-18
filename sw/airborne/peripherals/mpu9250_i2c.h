/*
 * Copyright (C) 2013 Gautier Hattenberger
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
 * @file peripherals/mpu9250_i2c.h
 *
 * Driver for the MPU-9250 using I2C.
 */

#ifndef MPU9250_I2C_H
#define MPU9250_I2C_H

#include "std.h"
#include "math/pprz_algebra_int.h"
#include "mcu_periph/i2c.h"

/* Include common MPU9250 options and definitions */
#include "peripherals/mpu9250.h"
#include "peripherals/ak8963.h"

#ifndef IMU_MPU9250_READ_MAG
#define IMU_MPU9250_READ_MAG TRUE
//the MPU6500 is the same as the 9250, except for that its lacking a magneto
#endif

#define MPU9250_BUFFER_EXT_LEN 16

enum Mpu9250I2cSlaveInitStatus {
  MPU9250_I2C_CONF_UNINIT,
  MPU9250_I2C_CONF_I2C_MST_DIS,
  MPU9250_I2C_CONF_I2C_BYPASS_EN,
  MPU9250_I2C_CONF_SLAVES_CONFIGURE,
  MPU9250_I2C_CONF_I2C_BYPASS_DIS,
  MPU9250_I2C_CONF_I2C_MST_CLK,
  MPU9250_I2C_CONF_I2C_MST_DELAY,
  MPU9250_I2C_CONF_I2C_SMPLRT,
  MPU9250_I2C_CONF_I2C_MST_EN,
  MPU9250_I2C_CONF_DONE
};

struct Mpu9250_I2c {
  struct i2c_periph *i2c_p;
  struct i2c_transaction i2c_trans;
  volatile bool data_available;     ///< data ready flag
  union {
    struct Int16Vect3 vect;           ///< accel data vector in accel coordinate system
    int16_t value[3];                 ///< accel data values accessible by channel index
  } data_accel;
  union {
    struct Int16Rates rates;          ///< rates data as angular rates in gyro coordinate system
    int16_t value[3];                 ///< rates data values accessible by channel index
  } data_rates;
  uint8_t data_ext[MPU9250_BUFFER_EXT_LEN];
  struct Mpu9250Config config;
  enum Mpu9250I2cSlaveInitStatus slave_init_status;
#ifdef IMU_MPU9250_READ_MAG
  struct Ak8963 akm;                  ///< "internal" magnetometer
#endif
};

// Functions
extern void mpu9250_i2c_init(struct Mpu9250_I2c *mpu, struct i2c_periph *i2c_p, uint8_t addr);
extern void mpu9250_i2c_start_configure(struct Mpu9250_I2c *mpu);
extern void mpu9250_i2c_read(struct Mpu9250_I2c *mpu);
extern void mpu9250_i2c_event(struct Mpu9250_I2c *mpu);

/// convenience function: read or start configuration if not already initialized
static inline void mpu9250_i2c_periodic(struct Mpu9250_I2c *mpu)
{
  if (mpu->config.initialized) {
    mpu9250_i2c_read(mpu);
  } else {
    mpu9250_i2c_start_configure(mpu);
  }
}

#endif // MPU9250_I2C_H
