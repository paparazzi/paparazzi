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
 * @file subsystems/imu/imu_altimu10.h
 *
 * Driver for the Pololu Altimu10 IMU.
 *
 * IMU with LSM6DS33, LIS3MDL and LPS25H via I2C.
 */

#include "subsystems/imu.h"
#include "subsystems/abi.h"
#include "mcu_periph/i2c.h"
#include "imu_altimu10.h"


/* i2c default */
#ifndef IMU_ALTIMU10_I2C_DEV
#define IMU_ALTIMU10_I2C_DEV i2c2
#endif

struct ImuAltimu10 imu_altimu10;

const float LSM6_ACCEL_SENS[4] = {
  LSM6_ACCEL_SENS_2G,
  LSM6_ACCEL_SENS_4G,
  LSM6_ACCEL_SENS_8G,
  LSM6_ACCEL_SENS_16G
};

const int32_t LSM6_ACCEL_SENS_FRAC[4][2] = {
  { LSM6_ACCEL_SENS_2G_NUM, LSM6_ACCEL_SENS_2G_DEN },
  { LSM6_ACCEL_SENS_4G_NUM, LSM6_ACCEL_SENS_4G_DEN },
  { LSM6_ACCEL_SENS_8G_NUM, LSM6_ACCEL_SENS_8G_DEN },
  { LSM6_ACCEL_SENS_16G_NUM, LSM6_ACCEL_SENS_16G_DEN }
};


const float LSM6_GYRO_SENS[4] = {
  LSM6_GYRO_SENS_245,
  LSM6_GYRO_SENS_500,
  LSM6_GYRO_SENS_1000,
  LSM6_GYRO_SENS_2000
};

const int32_t LSM6_GYRO_SENS_FRAC[4][2] = {
  { LSM6_GYRO_SENS_245_NUM, LSM6_GYRO_SENS_245_DEN },
  { LSM6_GYRO_SENS_500_NUM, LSM6_GYRO_SENS_500_DEN },
  { LSM6_GYRO_SENS_1000_NUM, LSM6_GYRO_SENS_1000_DEN },
  { LSM6_GYRO_SENS_2000_NUM, LSM6_GYRO_SENS_2000_DEN }
};

const float LIS3MDL_MAG_SENS[4] = {
  LIS3MDL_MAG_SENS_4G,
  LIS3MDL_MAG_SENS_8G,
  LIS3MDL_MAG_SENS_12G,
  LIS3MDL_MAG_SENS_16G
};

const int32_t LIS3MDL_MAG_SENS_FRAC[4][2] = {
  { LIS3MDL_MAG_SENS_4G_NUM, LIS3MDL_MAG_SENS_4G_DEN },
  { LIS3MDL_MAG_SENS_8G_NUM, LIS3MDL_MAG_SENS_8G_DEN },
  { LIS3MDL_MAG_SENS_12G_NUM, LIS3MDL_MAG_SENS_12G_DEN },
  { LIS3MDL_MAG_SENS_16G_NUM, LIS3MDL_MAG_SENS_16G_DEN }
};


void imu_altimu10_init(void)
{
  /* Set accel and gyro configuration */
  lsm6_i2c_init(&imu_altimu10.acc_g_lsm6, &(IMU_ALTIMU10_I2C_DEV), LSM6_ADDR);
  /* Set magneto configuration */
  lis3mdl_i2c_init(&imu_altimu10.mag_lis3mdl, &(IMU_ALTIMU10_I2C_DEV), LIS3MDL_ADDR);
  /* Set barometer configuration */
  lps25h_i2c_init(&imu_altimu10.baro_lps25h, &(IMU_ALTIMU10_I2C_DEV), LPS25H_ADDR);
}


void imu_altimu10_periodic(void)
{
  lsm6_i2c_periodic(&imu_altimu10.acc_g_lsm6);
  lis3mdl_i2c_periodic(&imu_altimu10.mag_lis3mdl);
  lps25h_i2c_periodic(&imu_altimu10.baro_lps25h);
}

void imu_altimu10_event(void)
{
  uint32_t now_ts = get_sys_time_usec();

  lsm6_i2c_event(&imu_altimu10.acc_g_lsm6);
  if (imu_altimu10.acc_g_lsm6.data_available) {

    imu.accel_unscaled.x = imu_altimu10.acc_g_lsm6.data_xl.vect.y;
    imu.accel_unscaled.y = imu_altimu10.acc_g_lsm6.data_xl.vect.x;
    imu.accel_unscaled.z = imu_altimu10.acc_g_lsm6.data_xl.vect.z;

    imu.gyro_unscaled.p = imu_altimu10.acc_g_lsm6.data_g.rates.p;
    imu.gyro_unscaled.q = imu_altimu10.acc_g_lsm6.data_g.rates.q;
    imu.gyro_unscaled.r = imu_altimu10.acc_g_lsm6.data_g.rates.r;

    imu_altimu10.acc_g_lsm6.data_available = false;
    imu_scale_accel(&imu);
    imu_scale_gyro(&imu);
    AbiSendMsgIMU_ACCEL_INT32(IMU_ALTIMU10_ID, now_ts, &imu.accel);
    AbiSendMsgIMU_GYRO_INT32(IMU_ALTIMU10_ID, now_ts, &imu.gyro);
  }

  lis3mdl_i2c_event(&imu_altimu10.mag_lis3mdl);
  if (imu_altimu10.mag_lis3mdl.data_available) {
    imu.mag_unscaled.x = -imu_altimu10.mag_lis3mdl.data.vect.x;
    imu.mag_unscaled.y = -imu_altimu10.mag_lis3mdl.data.vect.y;
    imu.mag_unscaled.z = -imu_altimu10.mag_lis3mdl.data.vect.z;

    imu_altimu10.mag_lis3mdl.data_available = false;
    imu_scale_mag(&imu);
    AbiSendMsgIMU_MAG_INT32(IMU_ALTIMU10_ID, now_ts, &imu.mag);
  }

  lps25h_i2c_event(&imu_altimu10.baro_lps25h);
  if (imu_altimu10.baro_lps25h.data_available) {
    imu_altimu10.baro_lps25h.data_available = false;
    AbiSendMsgBARO_ABS(IMU_ALTIMU10_ID, imu_altimu10.baro_lps25h.data);
  }
}
