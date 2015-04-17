/*
 * Copyright (C) 2012 Felix Ruess <felix.ruess@gmail.com
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

#include "subsystems/imu.h"
#include "subsystems/abi.h"
#include "generated/airframe.h"

#include "nps_sensors.h"

struct ImuNps imu_nps;

void imu_impl_init(void)
{

  imu_nps.gyro_available = FALSE;
  imu_nps.mag_available = FALSE;
  imu_nps.accel_available = FALSE;

}

void imu_periodic(void)
{
}


void imu_feed_gyro_accel(void)
{

  RATES_ASSIGN(imu.gyro_unscaled, sensors.gyro.value.x, sensors.gyro.value.y, sensors.gyro.value.z);
  VECT3_ASSIGN(imu.accel_unscaled, sensors.accel.value.x, sensors.accel.value.y, sensors.accel.value.z);

  // set availability flags...
  imu_nps.accel_available = TRUE;
  imu_nps.gyro_available = TRUE;

}


void imu_feed_mag(void)
{

  VECT3_ASSIGN(imu.mag_unscaled, sensors.mag.value.x, sensors.mag.value.y, sensors.mag.value.z);
  imu_nps.mag_available = TRUE;

}

void imu_nps_event(void)
{
  uint32_t now_ts = get_sys_time_usec();
  if (imu_nps.gyro_available) {
    imu_nps.gyro_available = FALSE;
    imu_scale_gyro(&imu);
    AbiSendMsgIMU_GYRO_INT32(IMU_BOARD_ID, now_ts, &imu.gyro);
  }
  if (imu_nps.accel_available) {
    imu_nps.accel_available = FALSE;
    imu_scale_accel(&imu);
    AbiSendMsgIMU_ACCEL_INT32(IMU_BOARD_ID, now_ts, &imu.accel);
  }
  if (imu_nps.mag_available) {
    imu_nps.mag_available = FALSE;
    imu_scale_mag(&imu);
    AbiSendMsgIMU_MAG_INT32(IMU_BOARD_ID, now_ts, &imu.mag);
  }
}
