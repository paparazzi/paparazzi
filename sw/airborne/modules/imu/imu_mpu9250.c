/*
 * Copyright (C) 2014 Gautier Hattenberger <gautier.hattenberger@enac.fr>
 *
 * This file is part of paparazzi
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
 * @file "modules/imu/imu_mpu9250.c"
 * @author Gautier Hattenberger
 *
 * Test module for the mpu9250
 */

#include "modules/imu/imu_mpu9250.h"

// Default I2C address
#ifndef IMU_MPU9250_ADDR
#define IMU_MPU9250_ADDR MPU9250_ADDR
#endif

struct Mpu9250_I2c mpu9250;

void imu_mpu9250_init(void)
{
  mpu9250_i2c_init(&mpu9250, &(IMU_MPU9250_I2C_DEV), IMU_MPU9250_ADDR);
}

void imu_mpu9250_periodic(void)
{
  mpu9250_i2c_periodic(&mpu9250);
}

void imu_mpu9250_event(void)
{
  mpu9250_i2c_event(&mpu9250);
}

#include "math/pprz_algebra_int.h"
#include "modules/datalink/downlink.h"
#include "modules/core/abi.h"

void imu_mpu9250_report(void)
{
  uint8_t id = IMU_MPU9250_ID;

  struct Int32Vect3 accel = {
    (int32_t)(mpu9250.data_accel.vect.x),
    (int32_t)(mpu9250.data_accel.vect.y),
    (int32_t)(mpu9250.data_accel.vect.z)
  };
  DOWNLINK_SEND_IMU_ACCEL_RAW(DefaultChannel, DefaultDevice, &id, &accel.x, &accel.y, &accel.z);

  struct Int32Rates rates = {
    (int32_t)(mpu9250.data_rates.rates.p),
    (int32_t)(mpu9250.data_rates.rates.q),
    (int32_t)(mpu9250.data_rates.rates.r)
  };
  DOWNLINK_SEND_IMU_GYRO_RAW(DefaultChannel, DefaultDevice, &id, &rates.p, &rates.q, &rates.r);

  struct Int32Vect3 mag = {
    (int32_t)(mpu9250.akm.data.vect.x),
    (int32_t)(mpu9250.akm.data.vect.y),
    (int32_t)(mpu9250.akm.data.vect.z)
  };
  DOWNLINK_SEND_IMU_MAG_RAW(DefaultChannel, DefaultDevice, &id, &mag.x, &mag.y, &mag.z);
}

