/*
 * Copyright (C) 2019 Gautier Hattenberger <gaurier.hattenberger@enac.fr>
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
 * @file peripherals/bmi088_regs.h
 *
 * Register and address definitions for BMI088.
 */

#ifndef BMI088_REGS_H
#define BMI088_REGS_H

/* default I2C address */
#define BMI088_ACCEL_ADDR             (0x18<<1)
#define BMI088_ACCEL_ADDR_ALT         (0x19<<1)

#define BMI088_ACCEL_CHIP_ID          0x00 // Default value 0x1E
#define BMI088_ACCEL_ERR_REG          0x02
#define BMI088_ACCEL_STATUS           0x03

#define BMI088_ACCEL_X_LSB            0x12
#define BMI088_ACCEL_X_MSB            0x13
#define BMI088_ACCEL_Y_LSB            0x14
#define BMI088_ACCEL_Y_MSB            0x15
#define BMI088_ACCEL_Z_LSB            0x16
#define BMI088_ACCEL_Z_MSB            0x17

#define BMI088_ACCEL_SENSOR_TIME_0    0x18
#define BMI088_ACCEL_SENSOR_TIME_1    0x19
#define BMI088_ACCEL_SENSOR_TIME_2    0x1A

#define BMI088_ACCEL_INT_STAT_1       0x1D

#define BMI088_ACCEL_TEMP_MSB         0x22
#define BMI088_ACCEL_TEMP_LSB         0x23

#define BMI088_ACCEL_CONF             0x40
#define BMI088_ACCEL_RANGE            0x41

#define BMI088_ACCEL_INT1_IO_CTRL     0x53
#define BMI088_ACCEL_INT2_IO_CTRL     0x54
#define BMI088_ACCEL_INT_MAP_DATA     0x58

#define BMI088_ACCEL_SELF_TEST        0x6D

#define BMI088_ACCEL_PWR_CONF         0x7C
#define BMI088_ACCEL_PWR_CTRl         0x7D

#define BMI088_ACCEL_SOFT_RESET       0x7E

#define BMI088_GYRO_ADDR                (0x68<<1)
#define BMI088_GYRO_ADDR_ALT            (0x69<<1)

#define BMI088_GYRO_CHIP_ID             0x00 // Default value 0x0F

#define BMI088_GYRO_RATE_X_LSB          0x02
#define BMI088_GYRO_RATE_X_MSB          0x03
#define BMI088_GYRO_RATE_Y_LSB          0x04
#define BMI088_GYRO_RATE_Y_MSB          0x05
#define BMI088_GYRO_RATE_Z_LSB          0x06
#define BMI088_GYRO_RATE_Z_MSB          0x07

#define BMI088_GYRO_INT_STAT_1          0x0A

#define BMI088_GYRO_RANGE               0x0F
#define BMI088_GYRO_BAND_WIDTH          0x10

#define BMI088_GYRO_LPM_1               0x11

#define BMI088_GYRO_SOFT_RESET          0x14

#define BMI088_GYRO_INT_CTRL            0x15
#define BMI088_GYRO_INT3_INT4_IO_CONF   0x16
#define BMI088_GYRO_INT3_INT4_IO_MAP    0x18

#define BMI088_GYRO_SELF_TEST           0x3C

/** Accel output range
 */
enum Bmi088AccelRanges
{
  BMI088_ACCEL_RANGE_3G = 0x00,
  BMI088_ACCEL_RANGE_6G = 0x01,
  BMI088_ACCEL_RANGE_12G = 0x02,
  BMI088_ACCEL_RANGE_24G = 0x03,
};

/** Accel outpur data rate
 */
enum Bmi088AccelODR
{
  BMI088_ACCEL_ODR_12 = 0x05,
  BMI088_ACCEL_ODR_25 = 0x06,
  BMI088_ACCEL_ODR_50 = 0x07,
  BMI088_ACCEL_ODR_100 = 0x08,
  BMI088_ACCEL_ODR_200 = 0x09,
  BMI088_ACCEL_ODR_400 = 0x0A,
  BMI088_ACCEL_ODR_800 = 0x0B,
  BMI088_ACCEL_ODR_1600 = 0x0C,
};

/** Accel bandwith
 */
enum Bmi088AccelBW
{
  BMI088_ACCEL_BW_OSR4 = 0x08,
  BMI088_ACCEL_BW_OSR2 = 0x09,
  BMI088_ACCEL_BW_NORMAL = 0x0A,
};

/** Accel power type
 */
enum Bmi088AccelPowerType
{
  BMI088_ACCEL_ACTIVE = 0x00,
  BMI088_ACCEL_SUSPEND = 0x03,
};

/** Accel power control
 */
enum Bmi088AccelPowerCtrl
{
  BMI088_ACCEL_POWER_OFF = 0x00,
  BMI088_ACCEL_POWER_ON = 0x04,
};

/** Gyro output range
 */
enum Bmi088GyroRanges
{
  BMI088_GYRO_RANGE_2000 = 0x00,
  BMI088_GYRO_RANGE_1000 = 0x01,
  BMI088_GYRO_RANGE_500 = 0x02,
  BMI088_GYRO_RANGE_250 = 0x03,
  BMI088_GYRO_RANGE_125 = 0x04,
};

/** Gyro output data rate and bandwidth
 */
enum Bmi088GyroODR
{
  BMI088_GYRO_ODR_2000_BW_532 = 0x00,
  BMI088_GYRO_ODR_2000_BW_230 = 0x01,
  BMI088_GYRO_ODR_1000_BW_116 = 0x02,
  BMI088_GYRO_ODR_400_BW_47 = 0x03,
  BMI088_GYRO_ODR_200_BW_23 = 0x04,
  BMI088_GYRO_ODR_100_BW_12 = 0x05,
  BMI088_GYRO_ODR_200_BW_64 = 0x06,
  BMI088_GYRO_ODR_100_BW_32 = 0x07,
};

/** Gyro power type
 */
enum Bmi088GyroPowerType
{
  BMI088_GYRO_NORMAL = 0x00,
  BMI088_GYRO_SUSPEND = 0x80,
  BMI088_GYRO_DEEP_SUSPEND = 0x20,
};


#endif /* BMI088_REGS_H */

