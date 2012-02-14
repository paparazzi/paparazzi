/*
 * $Id$
 *
 * Copyright (C) 2010 Antoine Drouin <poinix@gmail.com>
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

#ifndef IMU_ASPIRIN_H
#define IMU_ASPIRIN_H

#include "generated/airframe.h"
#include "subsystems/imu.h"


#ifdef IMU_ASPIRIN_VERSION_2_0
#define IMU_MAG_X_CHAN 2
#define IMU_MAG_Y_CHAN 0
#define IMU_MAG_Z_CHAN 1
#if !defined IMU_MAG_X_SIGN & !defined IMU_MAG_Y_SIGN & !defined IMU_MAG_Z_SIGN
#define IMU_MAG_X_SIGN 1
#define IMU_MAG_Y_SIGN 1
#define IMU_MAG_Z_SIGN 1
#endif
#endif

#if !defined IMU_GYRO_P_SIGN & !defined IMU_GYRO_Q_SIGN & !defined IMU_GYRO_R_SIGN
#define IMU_GYRO_P_SIGN   1
#define IMU_GYRO_Q_SIGN   1
#define IMU_GYRO_R_SIGN   1
#endif
#if !defined IMU_ACCEL_X_SIGN & !defined IMU_ACCEL_Y_SIGN & !defined IMU_ACCEL_Z_SIGN
#define IMU_ACCEL_X_SIGN  1
#define IMU_ACCEL_Y_SIGN  1
#define IMU_ACCEL_Z_SIGN  1
#endif

/** default gyro sensitivy and neutral from the datasheet
 * MPU60X0 has 16.4 LSB/(deg/s) at 2000deg/s range
 * sens = 1/16.4 * pi/180 * 2^INT32_RATE_FRAC
 * sens = 1/16.4 * pi/180 * 4096 = 4.359066229
 */
#if !defined IMU_GYRO_P_SENS & !defined IMU_GYRO_Q_SENS & !defined IMU_GYRO_R_SENS
#define IMU_GYRO_P_SENS 4.359
#define IMU_GYRO_P_SENS_NUM 4359
#define IMU_GYRO_P_SENS_DEN 1000
#define IMU_GYRO_Q_SENS 4.359
#define IMU_GYRO_Q_SENS_NUM 4359
#define IMU_GYRO_Q_SENS_DEN 1000
#define IMU_GYRO_R_SENS 4.359
#define IMU_GYRO_R_SENS_NUM 4359
#define IMU_GYRO_R_SENS_DEN 1000
#endif
#if !defined IMU_GYRO_P_NEUTRAL & !defined IMU_GYRO_Q_NEUTRAL & !defined IMU_GYRO_R_NEUTRAL
#define IMU_GYRO_P_NEUTRAL 0
#define IMU_GYRO_Q_NEUTRAL 0
#define IMU_GYRO_R_NEUTRAL 0
#endif

/** default accel sensitivy from the datasheet
 * MPU60X0 has 2048 LSB/g
 * fixed point sens: 9.81 [m/s^2] / 2048 [LSB/g] * 2^INT32_ACCEL_FRAC
 * sens = 9.81 / 2048 * 1024 = 4.905
 */
#if !defined IMU_ACCEL_X_SENS & !defined IMU_ACCEL_Y_SENS & !defined IMU_ACCEL_Z_SENS
#define IMU_ACCEL_X_SENS 4.905
#define IMU_ACCEL_X_SENS_NUM 4905
#define IMU_ACCEL_X_SENS_DEN 1000
#define IMU_ACCEL_Y_SENS 4.905
#define IMU_ACCEL_Y_SENS_NUM 4905
#define IMU_ACCEL_Y_SENS_DEN 1000
#define IMU_ACCEL_Z_SENS 4.905
#define IMU_ACCEL_Z_SENS_NUM 4905
#define IMU_ACCEL_Z_SENS_DEN 1000
#endif
#if !defined IMU_ACCEL_X_NEUTRAL & !defined IMU_ACCEL_Y_NEUTRAL & !defined IMU_ACCEL_Z_NEUTRAL
#define IMU_ACCEL_X_NEUTRAL 0
#define IMU_ACCEL_Y_NEUTRAL 0
#define IMU_ACCEL_Z_NEUTRAL 0
#endif


enum Aspirin2Status
  { Aspirin2StatusUninit,
    Aspirin2StatusIdle,
    Aspirin2StatusReading
  };

struct ImuAspirin2 {
  volatile enum Aspirin2Status status;
  volatile uint8_t imu_available;
  volatile uint8_t imu_tx_buf[64];
  volatile uint8_t imu_rx_buf[64];
  uint32_t time_since_last_reading;
};

extern struct ImuAspirin2 imu_aspirin2;


#define ASPIRIN2_TIMEOUT 3
/*

#define foo_handler() {}
#define ImuMagEvent(_mag_handler) {					\
      MagEvent(foo_handler); \
}


    if (hmc5843.data_available) {			\
      imu.mag_unscaled.x = hmc5843.data.value[IMU_MAG_X_CHAN];		\
      imu.mag_unscaled.y = hmc5843.data.value[IMU_MAG_Y_CHAN];		\
      imu.mag_unscaled.z = hmc5843.data.value[IMU_MAG_Z_CHAN];		\
      _mag_handler();							\
      hmc5843.data_available = FALSE;		\
    }									\
*/

/* underlying architecture */
#include "subsystems/imu/imu_aspirin2_arch.h"
/* must be implemented by underlying architecture */
extern void imu_aspirin2_arch_init(void);


static inline void imu_from_buff(void)
{
  int32_t x, y, z, p, q, r;


  // If the itg3200 I2C transaction has succeeded: convert the data
#define MPU_OFFSET_GYRO 10
  p = (int16_t) ((imu_aspirin2.imu_rx_buf[0+MPU_OFFSET_GYRO] << 8) | imu_aspirin2.imu_rx_buf[1+MPU_OFFSET_GYRO]);
  q = (int16_t) ((imu_aspirin2.imu_rx_buf[2+MPU_OFFSET_GYRO] << 8) | imu_aspirin2.imu_rx_buf[3+MPU_OFFSET_GYRO]);
  r = (int16_t) ((imu_aspirin2.imu_rx_buf[4+MPU_OFFSET_GYRO] << 8) | imu_aspirin2.imu_rx_buf[5+MPU_OFFSET_GYRO]);

#define MPU_OFFSET_ACC 2
  x = (int16_t) ((imu_aspirin2.imu_rx_buf[0+MPU_OFFSET_ACC] << 8) | imu_aspirin2.imu_rx_buf[1+MPU_OFFSET_ACC]);
  y = (int16_t) ((imu_aspirin2.imu_rx_buf[2+MPU_OFFSET_ACC] << 8) | imu_aspirin2.imu_rx_buf[3+MPU_OFFSET_ACC]);
  z = (int16_t) ((imu_aspirin2.imu_rx_buf[4+MPU_OFFSET_ACC] << 8) | imu_aspirin2.imu_rx_buf[5+MPU_OFFSET_ACC]);

#ifdef LISA_M_LONGITUDINAL_X
  RATES_ASSIGN(imu.gyro_unscaled, q, -p, r);
  VECT3_ASSIGN(imu.accel_unscaled, y, -x, z);
#else
  RATES_ASSIGN(imu.gyro_unscaled, p, q, r);
  VECT3_ASSIGN(imu.accel_unscaled, x, y, z);
#endif

  // Is this is new data
#define MPU_OFFSET_STATUS 1
  if (imu_aspirin2.imu_rx_buf[MPU_OFFSET_STATUS] & 0x01)
  {
    //gyr_valid = TRUE;
    //acc_valid = TRUE;
  }
  else
  {
  }
}


static inline void imu_aspirin2_event(void (* _gyro_handler)(void), void (* _accel_handler)(void), void (* _mag_handler)(void))
{
  if (imu_aspirin2.status == Aspirin2StatusUninit) return;

  // imu_aspirin2_arch_int_disable();

  if (imu_aspirin2.imu_available) 
  {
    imu_aspirin2.time_since_last_reading = 0;
    imu_aspirin2.imu_available = FALSE;
    imu_from_buff();

    _gyro_handler();
    _accel_handler();
  }
  // imu_aspirin2_arch_int_enable();

  // Reset everything if we've been waiting too long
  //if (imu_aspirin2.time_since_last_reading > ASPIRIN2_TIMEOUT) {
  //  imu_aspirin2.time_since_last_reading = 0;
  //  return;
  //}

}

#define ImuEvent(_gyro_handler, _accel_handler, _mag_handler) { \
  imu_aspirin2_event(_gyro_handler, _accel_handler, _mag_handler); \
}

#endif /* IMU_ASPIRIN_H */
