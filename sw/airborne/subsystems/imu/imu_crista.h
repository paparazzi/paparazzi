/*
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

#ifndef IMU_CRISTA_H
#define IMU_CRISTA_H

#include "subsystems/imu.h"
#include "generated/airframe.h"

#define ADS8344_NB_CHANNELS 8
extern uint16_t ADS8344_values[ADS8344_NB_CHANNELS];
extern volatile bool_t ADS8344_available;

#define ImuEvent(_gyro_handler, _accel_handler, _mag_handler) {   \
    if (ADS8344_available) {            \
      ADS8344_available = FALSE;          \
      imu.gyro_unscaled.p = ADS8344_values[IMU_GYRO_P_CHAN];  \
      imu.gyro_unscaled.q = ADS8344_values[IMU_GYRO_Q_CHAN];  \
      imu.gyro_unscaled.r = ADS8344_values[IMU_GYRO_R_CHAN];  \
      imu.accel_unscaled.x = ADS8344_values[IMU_ACCEL_X_CHAN];  \
      imu.accel_unscaled.y = ADS8344_values[IMU_ACCEL_Y_CHAN];  \
      imu.accel_unscaled.z = ADS8344_values[IMU_ACCEL_Z_CHAN];  \
      /* spare 3, temp 7 */           \
      _gyro_handler();            \
      _accel_handler();           \
    }                 \
    ImuMagEvent(_mag_handler);          \
  }

#ifdef USE_AMI601
#include "peripherals/ami601.h"
#define foo_handler() {}
#define ImuMagEvent(_mag_handler) {         \
    AMI601Event(foo_handler);           \
    if (ami601_status == AMI601_DATA_AVAILABLE) {     \
      imu.mag_unscaled.x = ami601_values[IMU_MAG_X_CHAN];   \
      imu.mag_unscaled.y = ami601_values[IMU_MAG_Y_CHAN];   \
      imu.mag_unscaled.z = ami601_values[IMU_MAG_Z_CHAN];   \
      ami601_status = AMI601_IDLE;          \
      _mag_handler();             \
    }                 \
  }
#elif defined USE_HMC5843
#include "peripherals/hmc5843.h"
#define foo_handler() {}
#define ImuMagEvent(_mag_handler) {         \
    MagEvent(foo_handler);            \
    if (hmc5843.data_available) {         \
      imu.mag_unscaled.x = hmc5843.data.value[IMU_MAG_X_CHAN];    \
      imu.mag_unscaled.y = hmc5843.data.value[IMU_MAG_Y_CHAN];    \
      imu.mag_unscaled.z = hmc5843.data.value[IMU_MAG_Z_CHAN];    \
      _mag_handler();             \
      hmc5843.data_available = FALSE;         \
    }                 \
  }
#else
#define ImuMagEvent(_mag_handler) {}
#endif

/* underlying architecture */
#include "subsystems/imu/imu_crista_arch.h"
/* must be defined by underlying architecture */
extern void imu_crista_arch_init(void);



#endif /* IMU_CRISTA_H */
