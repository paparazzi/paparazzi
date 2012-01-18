/*
 * $Id$
 *
 * Copyright (C) 2008-2009 Antoine Drouin <poinix@gmail.com>
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

#include "generated/airframe.h"

int imu_overrun = 0;

void imu_b2_arch_init(void) {

}

void imu_periodic(void) {

}

#ifdef USE_NPS
#include "nps_sensors.h"

void imu_feed_gyro_accel(void) {
  max1168_values[IMU_GYRO_P_CHAN]  = sensors.gyro.value.x;
  max1168_values[IMU_GYRO_Q_CHAN]  = sensors.gyro.value.y;
  max1168_values[IMU_GYRO_R_CHAN]  = sensors.gyro.value.z;
  max1168_values[IMU_ACCEL_X_CHAN] = sensors.accel.value.x;
  max1168_values[IMU_ACCEL_Y_CHAN] = sensors.accel.value.y;
  max1168_values[IMU_ACCEL_Z_CHAN] = sensors.accel.value.z;
  max1168_status = STA_MAX1168_DATA_AVAILABLE;
}


void imu_feed_mag(void) {
#if defined IMU_B2_MAG_TYPE && IMU_B2_MAG_TYPE == IMU_B2_MAG_MS2100
  ms2100_values[IMU_MAG_X_CHAN] = sensors.mag.value.x;
  ms2100_values[IMU_MAG_Y_CHAN] = sensors.mag.value.y;
  ms2100_values[IMU_MAG_Z_CHAN] = sensors.mag.value.z;
  ms2100_status = MS2100_DATA_AVAILABLE;
#elif defined IMU_B2_MAG_TYPE && IMU_B2_MAG_TYPE == IMU_B2_MAG_AMI601
  ami601_values[IMU_MAG_X_CHAN] = sensors.mag.value.x;
  ami601_values[IMU_MAG_Y_CHAN] = sensors.mag.value.y;
  ami601_values[IMU_MAG_Z_CHAN] = sensors.mag.value.z;
  ami601_status = AMI601_DATA_AVAILABLE;
#endif
}
#endif //USE_NPS
