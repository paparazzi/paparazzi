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

#include "modules/imu/imu_nps.h"
#include "modules/imu/imu.h"
#include "modules/core/abi.h"
#include "generated/airframe.h"
#include "modules/imu/imu_nps.h"
#include "nps_sensors.h"

struct ImuNps imu_nps;

void imu_nps_init(void)
{

  imu_nps.gyro_available = false;
  imu_nps.mag_available = false;
  imu_nps.accel_available = false;

  // Set the default scaling
  const struct Int32Rates gyro_scale[2] = {
    {NPS_GYRO_SENSITIVITY_NUM, NPS_GYRO_SENSITIVITY_NUM, NPS_GYRO_SENSITIVITY_NUM},
    {NPS_GYRO_SENSITIVITY_DEN, NPS_GYRO_SENSITIVITY_DEN, NPS_GYRO_SENSITIVITY_DEN}
  };
  const struct Int32Rates gyro_neutral = {
    NPS_GYRO_NEUTRAL_P, NPS_GYRO_NEUTRAL_Q, NPS_GYRO_NEUTRAL_R
  };
  const struct Int32Vect3 accel_scale[2] = {
    {NPS_ACCEL_SENSITIVITY_NUM, NPS_ACCEL_SENSITIVITY_NUM, NPS_ACCEL_SENSITIVITY_NUM},
    {NPS_ACCEL_SENSITIVITY_DEN, NPS_ACCEL_SENSITIVITY_DEN, NPS_ACCEL_SENSITIVITY_DEN}
  };
  const struct Int32Vect3 accel_neutral = {
    NPS_ACCEL_NEUTRAL_X, NPS_ACCEL_NEUTRAL_Y, NPS_ACCEL_NEUTRAL_Z
  };
  const struct Int32Vect3 mag_scale[2] = {
    {NPS_MAG_SENSITIVITY_NUM, NPS_MAG_SENSITIVITY_NUM, NPS_MAG_SENSITIVITY_NUM},
    {NPS_MAG_SENSITIVITY_DEN, NPS_MAG_SENSITIVITY_DEN, NPS_MAG_SENSITIVITY_DEN}
  };
  const struct Int32Vect3 mag_neutral = {
    NPS_MAG_NEUTRAL_X, NPS_MAG_NEUTRAL_Y, NPS_MAG_NEUTRAL_Z
  };
  imu_set_defaults_gyro(IMU_NPS_ID, NULL, &gyro_neutral, gyro_scale);
  imu_set_defaults_accel(IMU_NPS_ID, NULL, &accel_neutral, accel_scale);
  imu_set_defaults_mag(IMU_NPS_ID, NULL, &mag_neutral, mag_scale);
}


void imu_feed_gyro_accel(void)
{

  RATES_ASSIGN(imu_nps.gyro, sensors.gyro.value.x, sensors.gyro.value.y, sensors.gyro.value.z);
  VECT3_ASSIGN(imu_nps.accel, sensors.accel.value.x, sensors.accel.value.y, sensors.accel.value.z);

  // set availability flags...
  imu_nps.accel_available = true;
  imu_nps.gyro_available = true;

}


void imu_feed_mag(void)
{

  VECT3_ASSIGN(imu_nps.mag, sensors.mag.value.x, sensors.mag.value.y, sensors.mag.value.z);
  imu_nps.mag_available = true;

}

void imu_nps_event(void)
{
  uint32_t now_ts = get_sys_time_usec();
  if (imu_nps.gyro_available) {
    AbiSendMsgIMU_GYRO_RAW(IMU_NPS_ID, now_ts, &imu_nps.gyro, 1);
    imu_nps.gyro_available = false;
  }
  if (imu_nps.accel_available) {
    AbiSendMsgIMU_ACCEL_RAW(IMU_NPS_ID, now_ts, &imu_nps.accel, 1);
    imu_nps.accel_available = false;
  }
  if (imu_nps.mag_available) {
    AbiSendMsgIMU_MAG_RAW(IMU_NPS_ID, now_ts, &imu_nps.mag);
    imu_nps.mag_available = false;
  }
}
