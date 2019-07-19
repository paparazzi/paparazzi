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
 * @file peripherals/lsm6ds33.h
 *
 * LSM6DS33 accelerometer and gyrometer driver I2C interface.
 */

#ifndef LSM6_H
#define LSM6_H

#ifndef LSM6_XL_DEFAULT_ODR
#define LSM6_XL_DEFAULT_ODR (LSM6_ODR_XL_1_6KHZ << 4)
#endif

#ifndef LSM6_XL_DEFAULT_FS
#define LSM6_XL_DEFAULT_FS  (LSM6_FS_XL_16G << 2)
#endif


#ifndef LSM6_G_DEFAULT_ODR
#define LSM6_G_DEFAULT_ODR  (LSM6_ODR_G_1_6KHZ << 4)
#endif

#ifndef LSM6_G_DEFAULT_FS
#define LSM6_G_DEFAULT_FS   (LSM6_FS_G_1000 << 2)
#endif

#ifndef LSM6_C_DEFAULT
#define LSM6_C_DEFAULT      (0x04)
#endif

#ifndef LSM6_ORIENT
#define LSM6_ORIENT         (0x02)
#endif

#ifndef GYRO_SENS_H
#define GYRO_SENS_H
/** default gyro sensitivy from the datasheet
 * sens = [(deg/s)/LSB] * pi/180 * 2^INT32_RATE_FRAC
 * ex: LSM6 with 1000 deg/s has 0.035 (deg/s)/LSB
 *     sens = 0.035 * pi/180 * 4096 = 2.5021
 */

#define LSM6_GYRO_SENS_245 0.3127
#define LSM6_GYRO_SENS_245_NUM 3127
#define LSM6_GYRO_SENS_245_DEN 1000
#define LSM6_GYRO_SENS_500 0.6255
#define LSM6_GYRO_SENS_500_NUM 6255
#define LSM6_GYRO_SENS_500_DEN 1000
#define LSM6_GYRO_SENS_1000 2.5021
#define LSM6_GYRO_SENS_1000_NUM 25021
#define LSM6_GYRO_SENS_1000_DEN 1000
#define LSM6_GYRO_SENS_2000 5.0042
#define LSM6_GYRO_SENS_2000_NUM 50042
#define LSM6_GYRO_SENS_2000_DEN 1000

#endif

#ifndef ACCEL_SENS_H
#define ACCEL_SENS_H

#define LSM6_ACCEL_SENS_2G 0.613125
#define LSM6_ACCEL_SENS_2G_NUM 981
#define LSM6_ACCEL_SENS_2G_DEN 1600
#define LSM6_ACCEL_SENS_4G 1.22625
#define LSM6_ACCEL_SENS_4G_NUM 981
#define LSM6_ACCEL_SENS_4G_DEN 800
#define LSM6_ACCEL_SENS_8G 2.4525
#define LSM6_ACCEL_SENS_8G_NUM 981
#define LSM6_ACCEL_SENS_8G_DEN 400
#define LSM6_ACCEL_SENS_16G 4.905
#define LSM6_ACCEL_SENS_16G_NUM 981
#define LSM6_ACCEL_SENS_16G_DEN 200

#endif


#include "std.h"

/* Include address and register definition */
#include "peripherals/lsm6ds33_regs.h"

enum Lsm6ConfStatus {
    LSM6_CONF_UNINIT,
    LSM6_CONF_CTRL1_XL,
    LSM6_CONF_CTRL2_G,
    LSM6_CONF_CTRL3_C,
    LSM6_CONF_CTRL3_ORIENT,
    LSM6_CONF_DONE
};

struct Lsm6Config {
    uint8_t xl;     // Accelerometer configuration
    uint8_t g;      // Gyroscope configuration
    uint8_t c;      // Common configuration
    uint8_t orient; // Axes orientation

};

static inline void lsm6_set_default_config(struct Lsm6Config *c)
{
  c->xl  = LSM6_XL_DEFAULT_ODR | LSM6_XL_DEFAULT_FS;
  c->g   = LSM6_G_DEFAULT_ODR | LSM6_G_DEFAULT_FS;
  c->c   = LSM6_C_DEFAULT;
  c->orient = LSM6_ORIENT;
}


#endif // LSM6_H
