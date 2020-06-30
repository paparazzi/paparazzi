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
 * @file peripherals/bmi088.h
 *
 * BMI088 driver common interface (I2C and SPI).
 */

#ifndef BMI088_H
#define BMI088_H

#include "std.h"

/* Include address and register definition */
#include "peripherals/bmi088_regs.h"

/// Default gyro full scale range +- 1000°/s
#define BMI088_DEFAULT_GYRO_RANGE BMI088_GYRO_RANGE_1000
/// Default gyro output rate
#define BMI088_DEFAULT_GYRO_ODR BMI088_GYRO_ODR_1000_BW_116
/// Default accel full scale range +- 6g
#define BMI088_DEFAULT_ACCEL_RANGE BMI088_ACCEL_RANGE_12G
/// Default accel output rate
#define BMI088_DEFAULT_ACCEL_ODR BMI088_ACCEL_ODR_1600
/// Default accel bandwidth
#define BMI088_DEFAULT_ACCEL_BW BMI088_ACCEL_BW_OSR4

/** default gyro sensitivy from the datasheet
 * sens = 1/ [LSB/(deg/s)] * pi/180 * 2^INT32_RATE_FRAC
 * ex: BMI with 1000 deg/s has 32.8 LSB/(deg/s)
 *     sens = 1/32.8 * pi/180 * 4096 = 2.17953
 */
#define BMI088_GYRO_SENS_125 0.272442 // FIXME
#define BMI088_GYRO_SENS_125_NUM 19327
#define BMI088_GYRO_SENS_125_DEN 17735
#define BMI088_GYRO_SENS_250 0.544883
#define BMI088_GYRO_SENS_250_NUM 19327
#define BMI088_GYRO_SENS_250_DEN 35470
#define BMI088_GYRO_SENS_500 1.08977
#define BMI088_GYRO_SENS_500_NUM 57663
#define BMI088_GYRO_SENS_500_DEN 52913
#define BMI088_GYRO_SENS_1000 2.17953
#define BMI088_GYRO_SENS_1000_NUM 18271
#define BMI088_GYRO_SENS_1000_DEN 8383
#define BMI088_GYRO_SENS_2000 4.35906
#define BMI088_GYRO_SENS_2000_NUM 36542
#define BMI088_GYRO_SENS_2000_DEN 8383

// Get default sensitivity from a table
extern const float BMI088_GYRO_SENS[5];
// Get default sensitivity numerator and denominator from a table
extern const int32_t BMI088_GYRO_SENS_FRAC[5][2];

/** default accel sensitivy from the datasheet
 * sens = 9.81 [m/s^2] / [LSB/g] * 2^INT32_ACCEL_FRAC
 * ex: BMI with 6g has 5460 LSB/g
 *     sens = 9.81 [m/s^2] / 5460 [LSB/g] * 2^INT32_ACCEL_FRAC = 1.83982
 */
// FIXME
#define BMI088_ACCEL_SENS_3G 0.919912
#define BMI088_ACCEL_SENS_3G_NUM 9199
#define BMI088_ACCEL_SENS_3G_DEN 10000
#define BMI088_ACCEL_SENS_6G 1.83982
#define BMI088_ACCEL_SENS_6G_NUM 18398
#define BMI088_ACCEL_SENS_6G_DEN 10000
#define BMI088_ACCEL_SENS_12G 3.67965
#define BMI088_ACCEL_SENS_12G_NUM 36797
#define BMI088_ACCEL_SENS_12G_DEN 10000
#define BMI088_ACCEL_SENS_24G 7.3593
#define BMI088_ACCEL_SENS_24G_NUM 7359
#define BMI088_ACCEL_SENS_24G_DEN 1000

// Get default sensitivity from a table
extern const float BMI088_ACCEL_SENS[4];
// Get default sensitivity numerator and denominator from a table
extern const int32_t BMI088_ACCEL_SENS_FRAC[4][2];

enum Bmi088ConfStatus {
  BMI088_CONF_UNINIT,
  BMI088_CONF_ACCEL_RANGE,
  BMI088_CONF_ACCEL_ODR,
  BMI088_CONF_ACCEL_PWR_CONF,
  BMI088_CONF_ACCEL_PWR_CTRL,
  BMI088_CONF_GYRO_RANGE,
  BMI088_CONF_GYRO_ODR,
  BMI088_CONF_GYRO_PWR,
  BMI088_CONF_DONE
};

#define BMI088_CONFIG_ACCEL 0
#define BMI088_CONFIG_GYRO  1
/// Configuration function prototype
typedef void (*Bmi088ConfigSet)(void *bmi, uint8_t _reg, uint8_t _val, uint8_t _type);

struct Bmi088Config {
  enum Bmi088GyroRanges gyro_range;     ///< deg/s Range
  enum Bmi088GyroODR gyro_odr;          ///< output data rate
  enum Bmi088AccelRanges accel_range;   ///< g Range
  enum Bmi088AccelODR accel_odr;        ///< output data rate
  enum Bmi088AccelBW accel_bw;          ///< bandwidth
  enum Bmi088ConfStatus init_status;    ///< init status
  bool initialized;                     ///< config done flag
};

extern void bmi088_set_default_config(struct Bmi088Config *c);

/// Configuration sequence called once before normal use
extern void bmi088_send_config(Bmi088ConfigSet bmi_set, void *bmi, struct Bmi088Config *config);

#endif // BMI088_H
