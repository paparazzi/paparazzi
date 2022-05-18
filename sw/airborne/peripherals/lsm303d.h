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
 * @file peripherals/lsm303d.h
 *
 * Driver for ST LSM303D 3D accelerometer and magnetometer.
 */
#ifndef LSM303D_H
#define LSM303D_H

#include "std.h"
/* Address and register definitions */
#include "peripherals/lsm303d_regs.h"

/* LSM303D default conf */
#ifndef LSM303D_DEFAULT_AODR
#define LSM303D_DEFAULT_AODR (LSM303D_ACC_RATE_1600HZ << 4) //acc 1600 Hz
#endif

#ifndef LSM303D_DEFAULT_AFS
#define LSM303D_DEFAULT_AFS (LSM303D_ACC_RANGE_16G << 3) // acc +- 16G
#endif

#ifndef LSM303D_DEFAULT_MODR
#define LSM303D_DEFAULT_MODR (LSM303D_MAG_RATE_100HZ << 2) // Magneto Data Output Rate (100Hz)
#endif

#ifndef LSM303D_DEFAULT_MFS
#define LSM303D_DEFAULT_MFS (LSM303D_MAG_RANGE_2GAUSS << 5) // Magneto gain configuration (+/- 2 Gauss)
#endif

#ifndef LSM303D_DEFAULT_MD
#define LSM303D_DEFAULT_MD (LSM303D_MAG_MODE_CONTINOUS_CONVERSION << 0) // Magneto continious conversion mode
#endif

/** default accel sensitivy from the datasheet
 * LSM303DLHC has 732 LSB/g
 * fixed point sens: 9.81 [m/s^2] / 732 [LSB/g] * 2^INT32_ACCEL_FRAC
 * sens = 9.81 / 732 * 1024 = 13.72
 */
#define LSM303D_ACCEL_SENS_16G_NUM 13723
#define LSM303D_ACCEL_SENS_16G_DEN 1000

struct Lsm303dConfig {
  uint8_t acc_rate;    ///< Data Output Rate (Hz)
  uint8_t acc_scale;   ///< full scale selection (m/s²)

  uint8_t mag_rate;  ///< Data Output Rate Bits (Hz)
  uint8_t mag_scale;  ///< Full scale gain configuration (Gauss)
  uint8_t mag_mode;  ///< Measurement mode
};


/** config status states */
enum Lsm303dConfStatus {
  LSM303D_CONF_UNINIT,
  LSM303D_CONF_WHO_AM_I,
  LSM303D_CONF_CTRL_REG1,
  LSM303D_CONF_CTRL_REG2,
  LSM303D_CONF_CTRL_REG3,
  LSM303D_CONF_CTRL_REG4,
  LSM303D_CONF_CTRL_REG5,
  LSM303D_CONF_CTRL_REG6,
  LSM303D_CONF_CTRL_REG7,
  LSM303D_CONF_DONE
};

enum Lsm303dTarget {
  LSM303D_TARGET_ACC,
  LSM303D_TARGET_MAG
};

static inline void lsm303d_set_default_config(struct Lsm303dConfig *c)
{
  c->acc_rate = LSM303D_DEFAULT_AODR;
  c->acc_scale = LSM303D_DEFAULT_AFS;
  c->mag_rate = LSM303D_DEFAULT_MODR;
  c->mag_scale = LSM303D_DEFAULT_MFS;
  c->mag_mode = LSM303D_DEFAULT_MD;
}

#endif // LSM303D_H
