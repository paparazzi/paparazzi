/*
 * Copyright (C) 2011 Gautier Hattenberger
 * Derived from Aspirin and ppzuavimu drivers
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
 * @file boards/navgo/imu_navgo.c
 *
 * Driver for the IMU on the NavGo board.
 *
 *  - Gyroscope: Invensense ITG-3200
 *  - Accelerometer: Analog Devices ADXL345
 *  - Magnetometer: Honeywell HMC5883L
 */

#include <math.h>
#include "imu_navgo.h"
#include "mcu_periph/i2c.h"
#include "led.h"

// Downlink
#include "mcu_periph/uart.h"
#include "messages.h"
#include "subsystems/datalink/downlink.h"


#ifndef NAVGO_ACCEL_RATE
#define NAVGO_ACCEL_RATE ADXL345_RATE_25HZ
#endif
PRINT_CONFIG_VAR(NAVGO_ACCEL_RATE)

/* default gyro internal lowpass frequency and sample rate divider */
#if !defined NAVGO_GYRO_LOWPASS && !defined  NAVGO_GYRO_SMPLRT_DIV
#define NAVGO_GYRO_LOWPASS ITG3200_DLPF_10HZ
#define NAVGO_GYRO_SMPLRT_DIV 1
PRINT_CONFIG_MSG("Gyro output rate is 500Hz")
#endif
PRINT_CONFIG_VAR(NAVGO_GYRO_LOWPASS)
PRINT_CONFIG_VAR(NAVGO_GYRO_SMPLRT_DIV)

#if NAVGO_USE_MEDIAN_FILTER
#include "filters/median_filter.h"
struct MedianFilter3Int median_gyro, median_accel, median_mag;
#endif

struct ImuNavgo imu_navgo;

void imu_impl_init(void)
{
  /////////////////////////////////////////////////////////////////////
  // ITG3200
  itg3200_init(&imu_navgo.itg, &(IMU_NAVGO_I2C_DEV), ITG3200_ADDR_ALT);
  // change the default configuration
  imu_navgo.itg.config.smplrt_div = NAVGO_GYRO_SMPLRT_DIV;  // 500Hz sample rate since internal is 1kHz
  imu_navgo.itg.config.dlpf_cfg = NAVGO_GYRO_LOWPASS;

  /////////////////////////////////////////////////////////////////////
  // ADXL345
  adxl345_i2c_init(&imu_navgo.adxl, &(IMU_NAVGO_I2C_DEV), ADXL345_ADDR_ALT);
  // change the default data rate
  imu_navgo.adxl.config.rate = NAVGO_ACCEL_RATE;

  /////////////////////////////////////////////////////////////////////
  // HMC58XX
  hmc58xx_init(&imu_navgo.hmc, &(IMU_NAVGO_I2C_DEV), HMC58XX_ADDR);

#if NAVGO_USE_MEDIAN_FILTER
  // Init median filters
  InitMedianFilterRatesInt(median_gyro);
  InitMedianFilterVect3Int(median_accel);
  InitMedianFilterVect3Int(median_mag);
#endif

  imu_navgo.gyr_valid = FALSE;
  imu_navgo.acc_valid = FALSE;
  imu_navgo.mag_valid = FALSE;
}

void imu_periodic(void)
{
  // Start reading the latest gyroscope data
  itg3200_periodic(&imu_navgo.itg);

  // Start reading the latest accelerometer data
  // Periodicity is automatically adapted
  // 3200 is the maximum output freq corresponding to the parameter 0xF
  // A factor 2 is applied to reduice the delay without overloading the i2c
  RunOnceEvery((PERIODIC_FREQUENCY / (2 * 3200 >> (0xf - NAVGO_ACCEL_RATE))), adxl345_i2c_periodic(&imu_navgo.adxl));

  // Read HMC58XX at 100Hz (main loop for rotorcraft: 512Hz)
  RunOnceEvery(5, hmc58xx_periodic(&imu_navgo.hmc));

  //RunOnceEvery(20,imu_navgo_downlink_raw());
}


void imu_navgo_downlink_raw(void)
{
  DOWNLINK_SEND_IMU_GYRO_RAW(DefaultChannel, DefaultDevice, &imu.gyro_unscaled.p, &imu.gyro_unscaled.q,
                             &imu.gyro_unscaled.r);
  DOWNLINK_SEND_IMU_ACCEL_RAW(DefaultChannel, DefaultDevice, &imu.accel_unscaled.x, &imu.accel_unscaled.y,
                              &imu.accel_unscaled.z);
  DOWNLINK_SEND_IMU_MAG_RAW(DefaultChannel, DefaultDevice, &imu.mag_unscaled.x, &imu.mag_unscaled.y, &imu.mag_unscaled.z);
}

void imu_navgo_event(void)
{

  // If the itg3200 I2C transaction has succeeded: convert the data
  itg3200_event(&imu_navgo.itg);
  if (imu_navgo.itg.data_available) {
    RATES_ASSIGN(imu.gyro_unscaled, -imu_navgo.itg.data.rates.q, imu_navgo.itg.data.rates.p, imu_navgo.itg.data.rates.r);
#if NAVGO_USE_MEDIAN_FILTER
    UpdateMedianFilterRatesInt(median_gyro, imu.gyro_unscaled);
#endif
    imu_navgo.itg.data_available = FALSE;
    imu_navgo.gyr_valid = TRUE;
  }

  // If the adxl345 I2C transaction has succeeded: convert the data
  adxl345_i2c_event(&imu_navgo.adxl);
  if (imu_navgo.adxl.data_available) {
    VECT3_ASSIGN(imu.accel_unscaled, imu_navgo.adxl.data.vect.y, -imu_navgo.adxl.data.vect.x, imu_navgo.adxl.data.vect.z);
#if NAVGO_USE_MEDIAN_FILTER
    UpdateMedianFilterVect3Int(median_accel, imu.accel_unscaled);
#endif
    imu_navgo.adxl.data_available = FALSE;
    imu_navgo.acc_valid = TRUE;
  }

  // HMC58XX event task
  hmc58xx_event(&imu_navgo.hmc);
  if (imu_navgo.hmc.data_available) {
    VECT3_COPY(imu.mag_unscaled, imu_navgo.hmc.data.vect);
#if NAVGO_USE_MEDIAN_FILTER
    UpdateMedianFilterVect3Int(median_mag, imu.mag_unscaled);
#endif
    imu_navgo.hmc.data_available = FALSE;
    imu_navgo.mag_valid = TRUE;
  }

}

