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


#if USE_NPS
#include "nps_sensors.h"

void imu_feed_gyro_accel(void) {

#if 1
  // the high byte is in buf[0], low byte in buf[1], etc.
  imu_aspirin.i2c_trans_gyro.buf[0] = ((int16_t)sensors.gyro.value.x) >> 8;
  imu_aspirin.i2c_trans_gyro.buf[1] = ((int16_t)sensors.gyro.value.x) & 0xff;
  imu_aspirin.i2c_trans_gyro.buf[2] = ((int16_t)sensors.gyro.value.y) >> 8;
  imu_aspirin.i2c_trans_gyro.buf[3] = ((int16_t)sensors.gyro.value.y) & 0xff;
  imu_aspirin.i2c_trans_gyro.buf[4] = ((int16_t)sensors.gyro.value.z) >> 8;
  imu_aspirin.i2c_trans_gyro.buf[5] = ((int16_t)sensors.gyro.value.z) & 0xff;

  // low byte in buf[1], high byte in buf[2], etc...
  imu_aspirin.accel_rx_buf[1] = ((int16_t)sensors.accel.value.x) & 0xff;
  imu_aspirin.accel_rx_buf[2] = ((int16_t)sensors.accel.value.x) >> 8;
  imu_aspirin.accel_rx_buf[3] = ((int16_t)sensors.accel.value.y) & 0xff;
  imu_aspirin.accel_rx_buf[4] = ((int16_t)sensors.accel.value.y) >> 8;
  imu_aspirin.accel_rx_buf[5] = ((int16_t)sensors.accel.value.z) & 0xff;
  imu_aspirin.accel_rx_buf[6] = ((int16_t)sensors.accel.value.z) >> 8;

  // set availability flags...
  imu_aspirin.accel_available = true;

#else

  RATES_ASSIGN(imu.gyro_unscaled, sensors.gyro.value.x, sensors.gyro.value.y, sensors.gyro.value.z);
  VECT3_ASSIGN(imu.accel_unscaled, sensors.accel.value.x, sensors.accel.value.y, sensors.accel.value.z);

  // set availability flags...
  imu_aspirin.accel_available = true;
  imu_aspirin.gyro_available_blaaa = true;

#endif

}


void imu_feed_mag(void) {
  hmc5843.data.value[IMU_MAG_X_CHAN] = sensors.mag.value.x;
  hmc5843.data.value[IMU_MAG_Y_CHAN] = sensors.mag.value.y;
  hmc5843.data.value[IMU_MAG_Z_CHAN] = sensors.mag.value.z;
  hmc5843.data_available = TRUE;
}
#endif
