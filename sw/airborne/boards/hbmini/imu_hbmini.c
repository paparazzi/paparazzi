/*
 * Copyright (C) 2011 Paparazzi Team
 * Derived from Aspirin, NavGo and ppzuavimu drivers
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
 * @file boards/hbmini/imu_hbmini.c
 *
 * Driver for the IMU on the Hbmini board.
 *
 *  - Gyroscope: IDG 500 and ISZ 500, MAX1168
 *  - Accelerometer: Analog Devices ADXL320
 *  - Magnetometer: Honeywell HMC5883L
 */

#include <math.h>
#include "imu_hbmini.h"
#include "mcu_periph/i2c.h"
#include "led.h"
#include "subsystems/abi.h"

// Downlink
#include "mcu_periph/uart.h"
#include "messages.h"
#include "subsystems/datalink/downlink.h"


#if !defined IMU_GYRO_P_CHAN & !defined IMU_GYRO_Q_CHAN & !defined IMU_GYRO_R_CHAN
#define IMU_GYRO_P_CHAN  3
#define IMU_GYRO_Q_CHAN  4
#define IMU_GYRO_R_CHAN  5
#endif
#if !defined IMU_ACCEL_X_CHAN & !defined IMU_ACCEL_Y_CHAN & !defined IMU_ACCEL_Z_CHAN
#define IMU_ACCEL_X_CHAN 0
#define IMU_ACCEL_Y_CHAN 1
#define IMU_ACCEL_Z_CHAN 2
#endif
#if !defined IMU_MAG_X_CHAN & !defined IMU_MAG_Y_CHAN & !defined IMU_MAG_Z_CHAN
#define IMU_MAG_X_CHAN   2
#define IMU_MAG_Y_CHAN   0
#define IMU_MAG_Z_CHAN   1
#endif

struct ImuHbmini imu_hbmini;

void imu_impl_init(void)
{
  max1168_init();

  /////////////////////////////////////////////////////////////////////
  // HMC58XX
  hmc58xx_init(&imu_hbmini.hmc, &(IMU_HBMINI_I2C_DEV), HMC58XX_ADDR);
}

void imu_periodic(void)
{

  Max1168Periodic();

  // Read HMC58XX at 100Hz (main loop for rotorcraft: 512Hz)
  RunOnceEvery(5, hmc58xx_periodic(&imu_hbmini.hmc));

  //RunOnceEvery(20,imu_hbmini_downlink_raw());
}


void imu_hbmini_downlink_raw(void)
{
  DOWNLINK_SEND_IMU_GYRO_RAW(DefaultChannel, DefaultDevice, &imu.gyro_unscaled.p, &imu.gyro_unscaled.q,
                             &imu.gyro_unscaled.r);
  DOWNLINK_SEND_IMU_ACCEL_RAW(DefaultChannel, DefaultDevice, &imu.accel_unscaled.x, &imu.accel_unscaled.y,
                              &imu.accel_unscaled.z);
  DOWNLINK_SEND_IMU_MAG_RAW(DefaultChannel, DefaultDevice, &imu.mag_unscaled.x, &imu.mag_unscaled.y, &imu.mag_unscaled.z);
}

void imu_hbmini_event(void)
{
  uint32_t now_ts = get_sys_time_usec();

  max1168_event();

  if (max1168_status == MAX1168_DATA_AVAILABLE) {
    imu.gyro_unscaled.p  = max1168_values[IMU_GYRO_P_CHAN];
    imu.gyro_unscaled.q  = max1168_values[IMU_GYRO_Q_CHAN];
    imu.gyro_unscaled.r  = max1168_values[IMU_GYRO_R_CHAN];
    imu.accel_unscaled.x = max1168_values[IMU_ACCEL_X_CHAN];
    imu.accel_unscaled.y = max1168_values[IMU_ACCEL_Y_CHAN];
    imu.accel_unscaled.z = max1168_values[IMU_ACCEL_Z_CHAN];
    max1168_status = MAX1168_IDLE;
    imu_scale_gyro(&imu);
    imu_scale_accel(&imu);
    AbiSendMsgIMU_GYRO_INT32(IMU_BOARD_ID, now_ts, &imu.gyro);
    AbiSendMsgIMU_ACCEL_INT32(IMU_BOARD_ID, now_ts, &imu.accel);
  }

  // HMC58XX event task
  hmc58xx_event(&imu_hbmini.hmc);
  if (imu_hbmini.hmc.data_available) {
    imu.mag_unscaled.z = imu_hbmini.hmc.data.value[IMU_MAG_X_CHAN];
    imu.mag_unscaled.y = imu_hbmini.hmc.data.value[IMU_MAG_Y_CHAN];
    imu.mag_unscaled.x = imu_hbmini.hmc.data.value[IMU_MAG_Z_CHAN];

    imu_hbmini.hmc.data_available = FALSE;
    imu_scale_mag(&imu);
    AbiSendMsgIMU_MAG_INT32(IMU_BOARD_ID, now_ts, &imu.mag);
  }

}

