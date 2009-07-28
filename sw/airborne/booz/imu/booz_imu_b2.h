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

#ifndef BOOZ_IMU_B2_H
#define BOOZ_IMU_B2_H

#include "booz_imu.h"

#include "peripherals/booz_max1168.h"

#define IMU_B2_MAG_NONE   0
#define IMU_B2_MAG_MS2001 1
#define IMU_B2_MAG_AMI601 2

#if defined IMU_B2_MAG_TYPE && IMU_B2_MAG_TYPE == IMU_B2_MAG_MS2001
#include "peripherals/booz_ms2001.h"
#define BoozImuMagEvent(_mag_handler) {					\
    if (ms2001_status == MS2001_DATA_AVAILABLE) {			\
      booz_imu.mag_unscaled.x = ms2001_values[IMU_MAG_X_CHAN];		\
      booz_imu.mag_unscaled.y = ms2001_values[IMU_MAG_Y_CHAN];		\
      booz_imu.mag_unscaled.z = ms2001_values[IMU_MAG_Z_CHAN];		\
      ms2001_status = MS2001_IDLE;					\
      _mag_handler();							\
    }									\
  }
#elif defined IMU_B2_MAG_TYPE && IMU_B2_MAG_TYPE == IMU_B2_MAG_AMI601
#include "peripherals/booz_ami601.h"
#define foo_handler() {}
#define BoozImuMagEvent(_mag_handler) {					\
    AMI601Event(foo_handler);						\
    if (ami601_status == AMI601_DATA_AVAILABLE) {			\
      booz_imu.mag_unscaled.x = ami601_values[IMU_MAG_X_CHAN];		\
      booz_imu.mag_unscaled.y = ami601_values[IMU_MAG_Y_CHAN];		\
      booz_imu.mag_unscaled.z = ami601_values[IMU_MAG_Z_CHAN];		\
      ami601_status = AMI601_IDLE;					\
      _mag_handler();							\
    }									\
  }
#else
#define BoozImuMagEvent(_mag_handler) {}
#endif


#define BoozImuEvent(_gyro_accel_handler, _mag_handler) {		\
    if (booz_max1168_status == STA_MAX1168_DATA_AVAILABLE) {		\
      booz_imu.gyro_unscaled.p  = booz_max1168_values[IMU_GYRO_P_CHAN]; \
      booz_imu.gyro_unscaled.q  = booz_max1168_values[IMU_GYRO_Q_CHAN]; \
      booz_imu.gyro_unscaled.r  = booz_max1168_values[IMU_GYRO_R_CHAN]; \
      booz_imu.accel_unscaled.x = booz_max1168_values[IMU_ACCEL_X_CHAN]; \
      booz_imu.accel_unscaled.y = booz_max1168_values[IMU_ACCEL_Y_CHAN]; \
      booz_imu.accel_unscaled.z = booz_max1168_values[IMU_ACCEL_Z_CHAN]; \
      booz_max1168_status = STA_MAX1168_IDLE;				\
      _gyro_accel_handler();						\
    }									\
    BoozImuMagEvent(_mag_handler);					\
  }


/* underlying architecture */
#include "imu/booz_imu_b2_arch.h"
/* must be implemented by underlying architecture */
extern void booz_imu_b2_arch_init(void);


#endif /* BOOZ_IMU_B2_H */

