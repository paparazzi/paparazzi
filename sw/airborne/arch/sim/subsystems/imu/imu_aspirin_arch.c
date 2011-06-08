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

void imu_aspirin_arch_init(void) {

}

void imu_periodic(void) {

}

#include "nps_sensors.h"

void imu_feed_gyro_accel(void) {

  /* do something similar to this, fill imu_aspirin.i2c_trans_gyro and
   * imu_aspirin.accel_rx_buf
   */
  max1168_values[IMU_GYRO_P_CHAN]  = sensors.gyro.value.x;
  max1168_values[IMU_GYRO_Q_CHAN]  = sensors.gyro.value.y;
  max1168_values[IMU_GYRO_R_CHAN]  = sensors.gyro.value.z;
  max1168_values[IMU_ACCEL_X_CHAN] = sensors.accel.value.x;
  max1168_values[IMU_ACCEL_Y_CHAN] = sensors.accel.value.y;
  max1168_values[IMU_ACCEL_Z_CHAN] = sensors.accel.value.z;
  max1168_status = STA_MAX1168_DATA_AVAILABLE;
}


void imu_feed_mag(void) {
  hmc5843.data.value[IMU_MAG_X_CHAN] = sensors.mag.value.x;
  hmc5843.data.value[IMU_MAG_Y_CHAN] = sensors.mag.value.y;
  hmc5843.data.value[IMU_MAG_Z_CHAN] = sensors.mag.value.z;
  hmc5843.data_available = TRUE;
}
