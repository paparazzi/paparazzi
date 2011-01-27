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

#ifndef IMU_B2_H
#define IMU_B2_H

#include "subsystems/imu.h"
#include "generated/airframe.h"

#include "peripherals/max1168.h"

/* type of magnetometer */
#define IMU_B2_MAG_NONE   0
#define IMU_B2_MAG_MS2001 1
#define IMU_B2_MAG_AMI601 2


#ifdef IMU_B2_VERSION_1_0
/* Default IMU b2 sensors connection */
#if !defined IMU_GYRO_P_CHAN & !defined IMU_GYRO_Q_CHAN & !defined IMU_GYRO_R_CHAN
#define IMU_GYRO_P_CHAN  1
#define IMU_GYRO_Q_CHAN  0
#define IMU_GYRO_R_CHAN  2
#endif
#if !defined IMU_ACCEL_X_CHAN & !defined IMU_ACCEL_Y_CHAN & !defined IMU_ACCEL_Z_CHAN
#define IMU_ACCEL_X_CHAN 3
#define IMU_ACCEL_Y_CHAN 5
#define IMU_ACCEL_Z_CHAN 6
#endif
#if !defined IMU_MAG_X_CHAN & !defined IMU_MAG_Y_CHAN & !defined IMU_MAG_Z_CHAN
#define IMU_MAG_X_CHAN   0
#define IMU_MAG_Y_CHAN   1
#define IMU_MAG_Z_CHAN   2
#endif

#if !defined IMU_GYRO_P_SIGN & !defined IMU_GYRO_Q_SIGN & !defined IMU_GYRO_R_SIGN
#define IMU_GYRO_P_SIGN   1
#define IMU_GYRO_Q_SIGN  -1
#define IMU_GYRO_R_SIGN  -1
#endif
#if !defined IMU_ACCEL_X_SIGN & !defined IMU_ACCEL_Y_SIGN & !defined IMU_ACCEL_Z_SIGN
#define IMU_ACCEL_X_SIGN -1
#define IMU_ACCEL_Y_SIGN -1
#define IMU_ACCEL_Z_SIGN -1
#endif
#if !defined IMU_MAG_X_SIGN & !defined IMU_MAG_Y_SIGN & !defined IMU_MAG_Z_SIGN
#define IMU_MAG_X_SIGN    1
#define IMU_MAG_Y_SIGN   -1
#define IMU_MAG_Z_SIGN   -1
#endif
#endif /* IMU_B2_VERSION_1_0 */


#ifdef IMU_B2_VERSION_1_1
/* Default IMU b2 sensors connection */
#if !defined IMU_GYRO_P_CHAN & !defined IMU_GYRO_Q_CHAN & !defined IMU_GYRO_R_CHAN
#define IMU_GYRO_P_CHAN  1
#define IMU_GYRO_Q_CHAN  0
#define IMU_GYRO_R_CHAN  2
#endif
#if !defined IMU_ACCEL_X_CHAN & !defined IMU_ACCEL_Y_CHAN & !defined IMU_ACCEL_Z_CHAN
#define IMU_ACCEL_X_CHAN 5
#define IMU_ACCEL_Y_CHAN 3
#define IMU_ACCEL_Z_CHAN 4
#endif
#if !defined IMU_MAG_X_CHAN & !defined IMU_MAG_Y_CHAN & !defined IMU_MAG_Z_CHAN
#define IMU_MAG_X_CHAN   0
#define IMU_MAG_Y_CHAN   1
#define IMU_MAG_Z_CHAN   2
#endif

#if !defined IMU_GYRO_P_SIGN & !defined IMU_GYRO_Q_SIGN & !defined IMU_GYRO_R_SIGN
#define IMU_GYRO_P_SIGN   1
#define IMU_GYRO_Q_SIGN  -1
#define IMU_GYRO_R_SIGN  -1
#endif
#if !defined IMU_ACCEL_X_SIGN & !defined IMU_ACCEL_Y_SIGN & !defined IMU_ACCEL_Z_SIGN
#define IMU_ACCEL_X_SIGN -1
#define IMU_ACCEL_Y_SIGN -1
#define IMU_ACCEL_Z_SIGN -1
#endif
#if !defined IMU_MAG_X_SIGN & !defined IMU_MAG_Y_SIGN & !defined IMU_MAG_Z_SIGN
#define IMU_MAG_X_SIGN    1
#define IMU_MAG_Y_SIGN   -1
#define IMU_MAG_Z_SIGN   -1
#endif
#endif /* IMU_B2_VERSION_1_1 */

#ifdef IMU_B2_VERSION_1_2
/* Default IMU b2 sensors connection */
#if !defined IMU_GYRO_P_CHAN & !defined IMU_GYRO_Q_CHAN & !defined IMU_GYRO_R_CHAN
#define IMU_GYRO_P_CHAN  1
#define IMU_GYRO_Q_CHAN  0
#define IMU_GYRO_R_CHAN  2
#endif
#if !defined IMU_ACCEL_X_CHAN & !defined IMU_ACCEL_Y_CHAN & !defined IMU_ACCEL_Z_CHAN
#define IMU_ACCEL_X_CHAN 4
#define IMU_ACCEL_Y_CHAN 5
#define IMU_ACCEL_Z_CHAN 3
#endif
#if !defined IMU_MAG_X_CHAN & !defined IMU_MAG_Y_CHAN & !defined IMU_MAG_Z_CHAN
#define IMU_MAG_X_CHAN   0
#define IMU_MAG_Y_CHAN   1
#define IMU_MAG_Z_CHAN   2
#endif

#if !defined IMU_GYRO_P_SIGN & !defined IMU_GYRO_Q_SIGN & !defined IMU_GYRO_R_SIGN
#define IMU_GYRO_P_SIGN   1
#define IMU_GYRO_Q_SIGN  -1
#define IMU_GYRO_R_SIGN  -1
#endif
#if !defined IMU_ACCEL_X_SIGN & !defined IMU_ACCEL_Y_SIGN & !defined IMU_ACCEL_Z_SIGN
#define IMU_ACCEL_X_SIGN  1
#define IMU_ACCEL_Y_SIGN -1
#define IMU_ACCEL_Z_SIGN  1
#endif
#if !defined IMU_MAG_X_SIGN & !defined IMU_MAG_Y_SIGN & !defined IMU_MAG_Z_SIGN
#define IMU_MAG_X_SIGN    1
#define IMU_MAG_Y_SIGN   -1
#define IMU_MAG_Z_SIGN   -1
#endif
#endif /* IMU_B2_VERSION_1_2 */


#if defined IMU_B2_MAG_TYPE && IMU_B2_MAG_TYPE == IMU_B2_MAG_MS2001
#include "peripherals/ms2001.h"
#define ImuMagEvent(_mag_handler) {					\
    if (ms2001_status == MS2001_DATA_AVAILABLE) {			\
      imu.mag_unscaled.x = ms2001_values[IMU_MAG_X_CHAN];		\
      imu.mag_unscaled.y = ms2001_values[IMU_MAG_Y_CHAN];		\
      imu.mag_unscaled.z = ms2001_values[IMU_MAG_Z_CHAN];		\
      ms2001_status = MS2001_IDLE;					\
      _mag_handler();							\
    }									\
  }
#elif defined IMU_B2_MAG_TYPE && IMU_B2_MAG_TYPE == IMU_B2_MAG_AMI601
#include "peripherals/ami601.h"
#define foo_handler() {}
#define ImuMagEvent(_mag_handler) {					\
    AMI601Event(foo_handler);						\
    if (ami601_status == AMI601_DATA_AVAILABLE) {			\
      imu.mag_unscaled.x = ami601_values[IMU_MAG_X_CHAN];		\
      imu.mag_unscaled.y = ami601_values[IMU_MAG_Y_CHAN];		\
      imu.mag_unscaled.z = ami601_values[IMU_MAG_Z_CHAN];		\
      ami601_status = AMI601_IDLE;					\
      _mag_handler();							\
    }									\
  }
#elif defined IMU_B2_MAG_TYPE && IMU_B2_MAG_TYPE == IMU_B2_MAG_HMC5843
#include "peripherals/hmc5843.h"
#define foo_handler() {}
#define ImuMagEvent(_mag_handler) {					\
	  MagEvent(foo_handler); \
    if (hmc5843.data_available) {			\
      imu.mag_unscaled.x = hmc5843.data.value[IMU_MAG_X_CHAN];		\
      imu.mag_unscaled.y = hmc5843.data.value[IMU_MAG_Y_CHAN];		\
      imu.mag_unscaled.z = hmc5843.data.value[IMU_MAG_Z_CHAN];		\
      _mag_handler();							\
      hmc5843.data_available = FALSE;		\
    }									\
  }
#else
#define ImuMagEvent(_mag_handler) {}
#endif


#define ImuEvent(_gyro_handler, _accel_handler, _mag_handler) {		\
    if (max1168_status == STA_MAX1168_DATA_AVAILABLE) {		\
      imu.gyro_unscaled.p  = max1168_values[IMU_GYRO_P_CHAN]; \
      imu.gyro_unscaled.q  = max1168_values[IMU_GYRO_Q_CHAN]; \
      imu.gyro_unscaled.r  = max1168_values[IMU_GYRO_R_CHAN]; \
      imu.accel_unscaled.x = max1168_values[IMU_ACCEL_X_CHAN]; \
      imu.accel_unscaled.y = max1168_values[IMU_ACCEL_Y_CHAN]; \
      imu.accel_unscaled.z = max1168_values[IMU_ACCEL_Z_CHAN]; \
      max1168_status = STA_MAX1168_IDLE;				\
      _gyro_handler();						\
      _accel_handler();						\
    }									\
    ImuMagEvent(_mag_handler);					\
  }


/* underlying architecture */
#include "subsystems/imu/imu_b2_arch.h"
/* must be implemented by underlying architecture */
extern void imu_b2_arch_init(void);


#endif /* IMU_B2_H */
