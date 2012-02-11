/*
 * Copyright (C) 2010 The Paparazzi Team
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

#ifndef IMU_ANALOG_H
#define IMU_ANALOG_H


#define NB_ANALOG_IMU_ADC 6

// if not all gyros are used, override the ImuScaleGyro handler
#if defined ADC_CHANNEL_GYRO_P && defined ADC_CHANNEL_GYRO_Q && ! defined ADC_CHANNEL_GYRO_R

#define IMU_GYRO_R_NEUTRAL 0
#define ImuScaleGyro(_imu) {                                            \
    _imu.gyro.p = ((_imu.gyro_unscaled.p - _imu.gyro_neutral.p)*IMU_GYRO_P_SIGN*IMU_GYRO_P_SENS_NUM)/IMU_GYRO_P_SENS_DEN; \
    _imu.gyro.q = ((_imu.gyro_unscaled.q - _imu.gyro_neutral.q)*IMU_GYRO_Q_SIGN*IMU_GYRO_Q_SENS_NUM)/IMU_GYRO_Q_SENS_DEN; \
  }

#elif defined ADC_CHANNEL_GYRO_P && ! defined ADC_CHANNEL_GYRO_Q && ! defined ADC_CHANNEL_GYRO_R

#define IMU_GYRO_Q_NEUTRAL 0
#define IMU_GYRO_R_NEUTRAL 0
#define ImuScaleGyro(_imu) {                                            \
    _imu.gyro.p = ((_imu.gyro_unscaled.p - _imu.gyro_neutral.p)*IMU_GYRO_P_SIGN*IMU_GYRO_P_SENS_NUM)/IMU_GYRO_P_SENS_DEN; \
  }

#endif

// if we don't have any accelerometers, set an empty ImuScaleAccel handler
#if ! defined ADC_CHANNEL_ACCEL_X && ! defined ADC_CHANNEL_ACCEL_Z && ! defined ADC_CHANNEL_ACCEL_Z
#define ImuScaleAccel(_imu) {}
#endif

/*
 * we include imh.h after the definitions of ImuScale so we can override the default handlers
 */
#include "subsystems/imu.h"


extern volatile bool_t analog_imu_available;
extern int imu_overrun;

#define ImuEvent(_gyro_handler, _accel_handler, _mag_handler) {   \
    if (analog_imu_available) {                         \
      analog_imu_available = FALSE;                     \
      _gyro_handler();                            \
      _accel_handler();                            \
    }                                                   \
    ImuMagEvent(_mag_handler);                          \
  }

#define ImuMagEvent(_mag_handler) {             \
    if (0) {                                    \
      _mag_handler();                           \
    }                                           \
  }




#endif /* IMU_ANALOG_H */
