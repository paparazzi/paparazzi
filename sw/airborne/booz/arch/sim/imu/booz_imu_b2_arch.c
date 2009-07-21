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

#include "booz_imu.h"

#include "airframe.h"

void booz_imu_b2_arch_init(void) {

}

void booz_imu_periodic(void) {

}

#include "nps_sensors.h"

void booz_imu_feed_gyro_accel(void) {
  booz_max1168_values[IMU_GYRO_P_CHAN]  = sensors.gyro.value.x;
  booz_max1168_values[IMU_GYRO_Q_CHAN]  = sensors.gyro.value.y;
  booz_max1168_values[IMU_GYRO_R_CHAN]  = sensors.gyro.value.z;
  booz_max1168_values[IMU_ACCEL_X_CHAN] = sensors.accel.value.x;
  booz_max1168_values[IMU_ACCEL_Y_CHAN] = sensors.accel.value.y;
  booz_max1168_values[IMU_ACCEL_Z_CHAN] = sensors.accel.value.z;
  booz_max1168_status = STA_MAX1168_DATA_AVAILABLE;
}


void booz_imu_feed_mag(void) {
#if defined IMU_B2_MAG_TYPE && IMU_B2_MAG_TYPE == IMU_B2_MAG_MS2001
  ms2001_values[IMU_MAG_X_CHAN] = sensors.mag.value.x;
  ms2001_values[IMU_MAG_Y_CHAN] = sensors.mag.value.y;
  ms2001_values[IMU_MAG_Z_CHAN] = sensors.mag.value.z;
  ms2001_status = MS2001_DATA_AVAILABLE;
#elif defined IMU_B2_MAG_TYPE && IMU_B2_MAG_TYPE == IMU_B2_MAG_AMI601
  ami601_values[IMU_MAG_X_CHAN] = sensors.mag.value.x;
  ami601_values[IMU_MAG_Y_CHAN] = sensors.mag.value.y;
  ami601_values[IMU_MAG_Z_CHAN] = sensors.mag.value.z;
  ami601_status = AMI601_DATA_AVAILABLE;
#endif
}
