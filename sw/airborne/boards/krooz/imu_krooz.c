/*
 * Copyright (C) 2013 Sergey Krukowski <softsr@yahoo.de>
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
 * @file boards/krooz/imu_krooz.c
 *
 * Driver for the IMU on the KroozSD board.
 *
 * Invensense MPU-6050
 * Honeywell HMC-5883
 */

#include <math.h>
#include "boards/krooz/imu_krooz.h"
#include "modules/imu/imu_krooz_sd_arch.h"
#include "mcu_periph/i2c.h"
#include "led.h"
#include "filters/median_filter.h"
#include "modules/core/abi.h"

// Downlink
#include "mcu_periph/uart.h"
#include "pprzlink/messages.h"
#include "subsystems/datalink/downlink.h"

#if !defined KROOZ_LOWPASS_FILTER && !defined  KROOZ_SMPLRT_DIV
#define KROOZ_LOWPASS_FILTER MPU60X0_DLPF_256HZ
#define KROOZ_SMPLRT_DIV 1
#endif
PRINT_CONFIG_VAR(KROOZ_SMPLRT_DIV)
PRINT_CONFIG_VAR(KROOZ_LOWPASS_FILTER)

PRINT_CONFIG_VAR(KROOZ_GYRO_RANGE)
PRINT_CONFIG_VAR(KROOZ_ACCEL_RANGE)

struct ImuKrooz imu_krooz;

#if IMU_KROOZ_USE_ACCEL_MEDIAN_FILTER
struct MedianFilter3Int median_accel;
#endif
struct MedianFilter3Int median_mag;

void imu_krooz_init(void)
{
  /////////////////////////////////////////////////////////////////////
  // MPU-60X0
  mpu60x0_i2c_init(&imu_krooz.mpu, &(IMU_KROOZ_I2C_DEV), MPU60X0_ADDR);
  // change the default configuration
  imu_krooz.mpu.config.smplrt_div = KROOZ_SMPLRT_DIV;
  imu_krooz.mpu.config.dlpf_cfg = KROOZ_LOWPASS_FILTER;
  imu_krooz.mpu.config.gyro_range = KROOZ_GYRO_RANGE;
  imu_krooz.mpu.config.accel_range = KROOZ_ACCEL_RANGE;
  imu_krooz.mpu.config.drdy_int_enable = true;

  hmc58xx_init(&imu_krooz.hmc, &(IMU_KROOZ_I2C_DEV), HMC58XX_ADDR);

  // Init median filters
#if IMU_KROOZ_USE_ACCEL_MEDIAN_FILTER
  InitMedianFilterVect3Int(median_accel, MEDIAN_DEFAULT_SIZE);
#endif
  InitMedianFilterVect3Int(median_mag, MEDIAN_DEFAULT_SIZE);

  RATES_ASSIGN(imu_krooz.rates_sum, 0, 0, 0);
  VECT3_ASSIGN(imu_krooz.accel_sum, 0, 0, 0);
  imu_krooz.meas_nb = 0;

  imu_krooz.hmc_eoc = false;
  imu_krooz.mpu_eoc = false;

  imu_krooz_sd_arch_init();
}

void imu_krooz_periodic(void)
{
  // Start reading the latest gyroscope data
  if (!imu_krooz.mpu.config.initialized) {
    mpu60x0_i2c_start_configure(&imu_krooz.mpu);
  }

  if (!imu_krooz.hmc.initialized) {
    hmc58xx_start_configure(&imu_krooz.hmc);
  }

  if (imu_krooz.meas_nb) {
    RATES_ASSIGN(imu.gyro_unscaled, -imu_krooz.rates_sum.q / imu_krooz.meas_nb, imu_krooz.rates_sum.p / imu_krooz.meas_nb,
                 imu_krooz.rates_sum.r / imu_krooz.meas_nb);
    VECT3_ASSIGN(imu.accel_unscaled, -imu_krooz.accel_sum.y / imu_krooz.meas_nb, imu_krooz.accel_sum.x / imu_krooz.meas_nb,
                 imu_krooz.accel_sum.z / imu_krooz.meas_nb);

#if IMU_KROOZ_USE_ACCEL_MEDIAN_FILTER
    UpdateMedianFilterVect3Int(median_accel, imu.accel_unscaled);
#endif
    VECT3_SMUL(imu_krooz.accel_filtered, imu_krooz.accel_filtered, IMU_KROOZ_ACCEL_AVG_FILTER);
    VECT3_ADD(imu_krooz.accel_filtered, imu.accel_unscaled);
    VECT3_SDIV(imu_krooz.accel_filtered, imu_krooz.accel_filtered, (IMU_KROOZ_ACCEL_AVG_FILTER + 1));
    VECT3_COPY(imu.accel_unscaled, imu_krooz.accel_filtered);

    RATES_ASSIGN(imu_krooz.rates_sum, 0, 0, 0);
    VECT3_ASSIGN(imu_krooz.accel_sum, 0, 0, 0);
    imu_krooz.meas_nb = 0;

    uint32_t now_ts = get_sys_time_usec();
    imu_scale_gyro(&imu);
    imu_scale_accel(&imu);
    AbiSendMsgIMU_GYRO_INT32(IMU_BOARD_ID, now_ts, &imu.gyro);
    AbiSendMsgIMU_ACCEL_INT32(IMU_BOARD_ID, now_ts, &imu.accel);
  }

  //RunOnceEvery(10,imu_krooz_downlink_raw());
}

void imu_krooz_downlink_raw(void)
{
  DOWNLINK_SEND_IMU_GYRO_RAW(DefaultChannel, DefaultDevice, &imu.gyro_unscaled.p, &imu.gyro_unscaled.q,
                             &imu.gyro_unscaled.r);
  DOWNLINK_SEND_IMU_ACCEL_RAW(DefaultChannel, DefaultDevice, &imu.accel_unscaled.x, &imu.accel_unscaled.y,
                              &imu.accel_unscaled.z);
}


void imu_krooz_event(void)
{
  if (imu_krooz.mpu_eoc) {
    mpu60x0_i2c_read(&imu_krooz.mpu);
    imu_krooz.mpu_eoc = false;
  }

  // If the MPU6050 I2C transaction has succeeded: convert the data
  mpu60x0_i2c_event(&imu_krooz.mpu);
  if (imu_krooz.mpu.data_available) {
    RATES_ADD(imu_krooz.rates_sum, imu_krooz.mpu.data_rates.rates);
    VECT3_ADD(imu_krooz.accel_sum, imu_krooz.mpu.data_accel.vect);
    imu_krooz.meas_nb++;
    imu_krooz.mpu.data_available = false;
  }

  if (imu_krooz.hmc_eoc) {
    hmc58xx_read(&imu_krooz.hmc);
    imu_krooz.hmc_eoc = false;
  }

  // If the HMC5883 I2C transaction has succeeded: convert the data
  hmc58xx_event(&imu_krooz.hmc);
  if (imu_krooz.hmc.data_available) {
    VECT3_ASSIGN(imu.mag_unscaled, imu_krooz.hmc.data.vect.y, -imu_krooz.hmc.data.vect.x, imu_krooz.hmc.data.vect.z);
    UpdateMedianFilterVect3Int(median_mag, imu.mag_unscaled);
    imu_krooz.hmc.data_available = false;
    imu_scale_mag(&imu);
    AbiSendMsgIMU_MAG_INT32(IMU_BOARD_ID, get_sys_time_usec(), &imu.mag);
  }
}
