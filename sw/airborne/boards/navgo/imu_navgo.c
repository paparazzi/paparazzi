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
 *
 */

#include <math.h>
#include "imu_navgo.h"
#include "mcu_periph/i2c.h"
#include "led.h"

// Downlink
#include "mcu_periph/uart.h"
#include "messages.h"
#include "subsystems/datalink/downlink.h"

#ifndef DOWNLINK_DEVICE
#define DOWNLINK_DEVICE DOWNLINK_AP_DEVICE
#endif

// Peripherials

// Configure ITG3200
// ITG3200_I2C_DEVICE IMU_UMARIM_I2C_DEVICE
// ITG3200_I2C_ADDR ITG3200_ADDR_ALT
// ITG3200_SMPLRT_DIV 1
#include "peripherals/itg3200.extra.h"

// Configure ADXL345
// ADXL345_I2C_DEVICE IMU_UMARIM_I2C_DEVICE
// ADXL345_I2C_ADDR ADXL345_ADDR_ALT
#include "peripherals/adxl345.extra_i2c.h"

// Configure HMC58XX
#include "peripherals/hmc58xx.h"

// Results
volatile bool_t gyr_valid;
volatile bool_t acc_valid;
volatile bool_t mag_valid;

void imu_impl_init(void)
{
  /////////////////////////////////////////////////////////////////////
  // ITG3200
  itg3200_init();

  /////////////////////////////////////////////////////////////////////
  // ADXL345
  adxl345_init();

  /////////////////////////////////////////////////////////////////////
  // HMC58XX
  hmc58xx_init();
}

void imu_periodic( void )
{
  // Start reading the latest gyroscope data
  Itg3200Periodic();

  // Start reading the latest accelerometer data
  // Periodicity is automatically adapted
  // 3200 is the maximum output freq corresponding to the parameter 0xF
  // A factor 2 is applied to reduice the delay without overloading the i2c
  RunOnceEvery((PERIODIC_FREQUENCY/(2*3200>>(0xf-ADXL345_BW_RATE))),Adxl345Periodic());

  // Read HMC58XX at 100Hz (main loop for rotorcraft: 512Hz)
  RunOnceEvery(5,Hmc58xxPeriodic());

  //RunOnceEvery(20,imu_navgo_downlink_raw());
}


void imu_navgo_downlink_raw( void )
{
  DOWNLINK_SEND_IMU_GYRO_RAW(DefaultChannel, DefaultDevice,&imu.gyro_unscaled.p,&imu.gyro_unscaled.q,&imu.gyro_unscaled.r);
  DOWNLINK_SEND_IMU_ACCEL_RAW(DefaultChannel, DefaultDevice,&imu.accel_unscaled.x,&imu.accel_unscaled.y,&imu.accel_unscaled.z);
  DOWNLINK_SEND_IMU_MAG_RAW(DefaultChannel, DefaultDevice,&imu.mag_unscaled.x,&imu.mag_unscaled.y,&imu.mag_unscaled.z);
}


void imu_navgo_event( void )
{

  // If the itg3200 I2C transaction has succeeded: convert the data
  itg3200_event();
  if (itg3200_data_available) {
    RATES_ASSIGN(imu.gyro_unscaled, -itg3200_data.q, itg3200_data.p, itg3200_data.r);
    itg3200_data_available = FALSE;
    gyr_valid = TRUE;
  }

  // If the adxl345 I2C transaction has succeeded: convert the data
  adxl345_event();
  if (adxl345_data_available) {
    VECT3_ASSIGN(imu.accel_unscaled, adxl345_data.y, -adxl345_data.x, adxl345_data.z);
    adxl345_data_available = FALSE;
    acc_valid = TRUE;
  }

  // HMC58XX event task
  hmc58xx_event();
  if (hmc58xx_data_available) {
    VECT3_ASSIGN(imu.mag_unscaled, hmc58xx_data.x, hmc58xx_data.y, hmc58xx_data.z);
    hmc58xx_data_available = FALSE;
    mag_valid = TRUE;
  }

}

