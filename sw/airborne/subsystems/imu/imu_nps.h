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

#ifndef IMU_NPS_H
#define IMU_NPS_H

#include "subsystems/imu.h"

#include "generated/airframe.h"

/** we just define some defaults for aspirin v1.5 for now
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


/** we just define some defaults for aspirin v1.5 for now
 */
#if !defined IMU_ACCEL_X_SENS & !defined IMU_ACCEL_Y_SENS & !defined IMU_ACCEL_Z_SENS
#define IMU_ACCEL_X_SENS 37.91
#define IMU_ACCEL_X_SENS_NUM 3791
#define IMU_ACCEL_X_SENS_DEN 100
#define IMU_ACCEL_Y_SENS 37.91
#define IMU_ACCEL_Y_SENS_NUM 3791
#define IMU_ACCEL_Y_SENS_DEN 100
#define IMU_ACCEL_Z_SENS 39.24
#define IMU_ACCEL_Z_SENS_NUM 3924
#define IMU_ACCEL_Z_SENS_DEN 100
#endif


#if !defined IMU_MAG_X_SENS & !defined IMU_MAG_Y_SENS & !defined IMU_MAG_Z_SENS
#define IMU_MAG_X_SENS 3.5
#define IMU_MAG_X_SENS_NUM 7
#define IMU_MAG_X_SENS_DEN 2
#define IMU_MAG_Y_SENS 3.5
#define IMU_MAG_Y_SENS_NUM 7
#define IMU_MAG_Y_SENS_DEN 2
#define IMU_MAG_Z_SENS 3.5
#define IMU_MAG_Z_SENS_NUM 7
#define IMU_MAG_Z_SENS_DEN 2
#endif


struct ImuNps {
  uint8_t mag_available;
  uint8_t accel_available;
  uint8_t gyro_available;
};

extern struct ImuNps imu_nps;

extern void imu_feed_gyro_accel(void);
extern void imu_feed_mag(void);

#define ImuMagEvent(_mag_handler) {                             \
    if (imu_nps.mag_available) {                                \
      imu_nps.mag_available = FALSE;                            \
      _mag_handler();                                           \
    }                                                           \
  }

#define ImuEvent(_gyro_handler, _accel_handler, _mag_handler) { \
    if (imu_nps.accel_available) {                              \
      imu_nps.accel_available = FALSE;                          \
      _accel_handler();                                         \
    }                                                           \
    if (imu_nps.gyro_available) {                               \
      imu_nps.gyro_available = FALSE;                           \
      _gyro_handler();                                          \
    }                                                           \
    ImuMagEvent(_mag_handler);                                  \
  }

#define ImuScaleGyro(_imu) {                                            \
    RATES_COPY(_imu.gyro_prev, _imu.gyro);                              \
    _imu.gyro.p = ((_imu.gyro_unscaled.p - _imu.gyro_neutral.p) * IMU_GYRO_P_SENS_NUM) / IMU_GYRO_P_SENS_DEN; \
    _imu.gyro.q = ((_imu.gyro_unscaled.q - _imu.gyro_neutral.q) * IMU_GYRO_Q_SENS_NUM) / IMU_GYRO_Q_SENS_DEN; \
    _imu.gyro.r = ((_imu.gyro_unscaled.r - _imu.gyro_neutral.r) * IMU_GYRO_R_SENS_NUM) / IMU_GYRO_R_SENS_DEN; \
  }

#define ImuScaleAccel(_imu) {                                           \
    VECT3_COPY(_imu.accel_prev, _imu.accel);                            \
    _imu.accel.x = ((_imu.accel_unscaled.x - _imu.accel_neutral.x) * IMU_ACCEL_X_SENS_NUM) / IMU_ACCEL_X_SENS_DEN; \
    _imu.accel.y = ((_imu.accel_unscaled.y - _imu.accel_neutral.y) * IMU_ACCEL_Y_SENS_NUM) / IMU_ACCEL_Y_SENS_DEN; \
    _imu.accel.z = ((_imu.accel_unscaled.z - _imu.accel_neutral.z) * IMU_ACCEL_Z_SENS_NUM) / IMU_ACCEL_Z_SENS_DEN; \
  }

#define ImuScaleMag(_imu) {                                             \
    _imu.mag.x = ((_imu.mag_unscaled.x - _imu.mag_neutral.x) * IMU_MAG_X_SENS_NUM) / IMU_MAG_X_SENS_DEN; \
    _imu.mag.y = ((_imu.mag_unscaled.y - _imu.mag_neutral.y) * IMU_MAG_Y_SENS_NUM) / IMU_MAG_Y_SENS_DEN; \
    _imu.mag.z = ((_imu.mag_unscaled.z - _imu.mag_neutral.z) * IMU_MAG_Z_SENS_NUM) / IMU_MAG_Z_SENS_DEN; \
  }

#endif /* IMU_NPS_H */
