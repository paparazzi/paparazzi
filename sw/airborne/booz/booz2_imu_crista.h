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

#ifndef BOOZ2_IMU_CRISTA_H
#define BOOZ2_IMU_CRISTA_H

#include "booz2_imu.h"
#include "booz2_imu_crista_hw.h"


extern void booz2_imu_impl_init(void);
extern void booz2_imu_periodic(void);

#define ADS8344_NB_CHANNELS 8
extern uint16_t ADS8344_values[ADS8344_NB_CHANNELS];
extern bool_t ADS8344_available;

#define Booz2ImuEvent(_gyro_accel_handler, _mag_handler) {		\
    if (ADS8344_available) {						\
      ADS8344_available = FALSE;					\
      booz_imu.gyro_unscaled.p = ADS8344_values[IMU_GYRO_P_CHAN];	\
      booz_imu.gyro_unscaled.q = ADS8344_values[IMU_GYRO_Q_CHAN];	\
      booz_imu.gyro_unscaled.r = ADS8344_values[IMU_GYRO_R_CHAN];	\
      booz_imu.accel_unscaled.x = ADS8344_values[IMU_ACCEL_X_CHAN];	\
      booz_imu.accel_unscaled.y = ADS8344_values[IMU_ACCEL_Y_CHAN];	\
      booz_imu.accel_unscaled.z = ADS8344_values[IMU_ACCEL_Z_CHAN];	\
      /* spare 3, temp 7 */						\
      _gyro_accel_handler();						\
    }									\
    Booz2ImuMagEvent(_mag_handler);					\
  }

#ifdef USE_AMI601
#include "AMI601.h"
#define foo_handler() {}
#define Booz2ImuMagEvent(_mag_handler) {				\
    AMI601Event(foo_handler);						\
    if (ami601_status == AMI601_DATA_AVAILABLE) {			\
      booz_imu.mag_unscaled.x = ami601_val[IMU_MAG_X_CHAN];		\
      booz_imu.mag_unscaled.y = ami601_val[IMU_MAG_Y_CHAN];		\
      booz_imu.mag_unscaled.z = ami601_val[IMU_MAG_Z_CHAN];		\
      ami601_status = AMI601_IDLE;					\
      _mag_handler();							\
    }									\
  }
#else
#define Booz2ImuMagEvent(_mag_handler) {}
#endif

#endif /* BOOZ2_IMU_CRISTA_H */

