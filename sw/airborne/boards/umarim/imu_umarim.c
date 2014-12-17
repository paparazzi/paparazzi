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
 * @file boards/umarim/imu_umarim.c
 *
 * Driver for the IMU on the Umarim board.
 *
 *  - Gyroscope: Invensense ITG-3200
 *  - Accelerometer: Analog Devices ADXL345
 */

#include <math.h>
#include "imu_umarim.h"
#include "mcu_periph/i2c.h"
#include "generated/airframe.h"

// Downlink
#include "mcu_periph/uart.h"
#include "messages.h"
#include "subsystems/datalink/downlink.h"


#ifndef UMARIM_ACCEL_RANGE
#define UMARIM_ACCEL_RANGE ADXL345_RANGE_16G
#endif
PRINT_CONFIG_VAR(UMARIM_ACCEL_RANGE)

#ifndef UMARIM_ACCEL_RATE
#define UMARIM_ACCEL_RATE ADXL345_RATE_50HZ
#endif
PRINT_CONFIG_VAR(UMARIM_ACCEL_RATE)


/* default gyro internal lowpass frequency and sample rate divider */
#if !defined UMARIM_GYRO_LOWPASS && !defined  UMARIM_GYRO_SMPLRT_DIV
#define UMARIM_GYRO_LOWPASS ITG3200_DLPF_20HZ
#define UMARIM_GYRO_SMPLRT_DIV 19
PRINT_CONFIG_MSG("Gyro output rate is 50Hz")
#endif
PRINT_CONFIG_VAR(UMARIM_GYRO_LOWPASS)
PRINT_CONFIG_VAR(UMARIM_GYRO_SMPLRT_DIV)

struct ImuUmarim imu_umarim;

void imu_impl_init(void)
{
  /////////////////////////////////////////////////////////////////////
  // ITG3200
  itg3200_init(&imu_umarim.itg, &(IMU_UMARIM_I2C_DEV), ITG3200_ADDR_ALT);
  // change the default configuration
  imu_umarim.itg.config.smplrt_div = UMARIM_GYRO_SMPLRT_DIV;
  imu_umarim.itg.config.dlpf_cfg = UMARIM_GYRO_LOWPASS;

  /////////////////////////////////////////////////////////////////////
  // ADXL345
  adxl345_i2c_init(&imu_umarim.adxl, &(IMU_UMARIM_I2C_DEV), ADXL345_ADDR_ALT);
  // change the default data rate
  imu_umarim.adxl.config.rate = UMARIM_ACCEL_RATE;
  imu_umarim.adxl.config.range = UMARIM_ACCEL_RANGE;

  imu_umarim.gyr_valid = FALSE;
  imu_umarim.acc_valid = FALSE;
}

void imu_periodic(void)
{
  // Start reading the latest gyroscope data
  itg3200_periodic(&imu_umarim.itg);

  // Start reading the latest accelerometer data
  adxl345_i2c_periodic(&imu_umarim.adxl);

  //RunOnceEvery(10,imu_umarim_downlink_raw());
}

void imu_umarim_downlink_raw(void)
{
  DOWNLINK_SEND_IMU_GYRO_RAW(DefaultChannel, DefaultDevice, &imu.gyro_unscaled.p, &imu.gyro_unscaled.q,
                             &imu.gyro_unscaled.r);
  DOWNLINK_SEND_IMU_ACCEL_RAW(DefaultChannel, DefaultDevice, &imu.accel_unscaled.x, &imu.accel_unscaled.y,
                              &imu.accel_unscaled.z);
}


void imu_umarim_event(void)
{
  // If the itg3200 I2C transaction has succeeded: convert the data
  itg3200_event(&imu_umarim.itg);
  if (imu_umarim.itg.data_available) {
    RATES_COPY(imu.gyro_unscaled, imu_umarim.itg.data.rates);
    imu_umarim.itg.data_available = FALSE;
    imu_umarim.gyr_valid = TRUE;
  }

  // If the adxl345 I2C transaction has succeeded: convert the data
  adxl345_i2c_event(&imu_umarim.adxl);
  if (imu_umarim.adxl.data_available) {
    VECT3_ASSIGN(imu.accel_unscaled, imu_umarim.adxl.data.vect.y, -imu_umarim.adxl.data.vect.x,
                 imu_umarim.adxl.data.vect.z);
    imu_umarim.adxl.data_available = FALSE;
    imu_umarim.acc_valid = TRUE;
  }
}

