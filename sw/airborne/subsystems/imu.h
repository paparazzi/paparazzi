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

#ifndef IMU_H
#define IMU_H

#include "math/pprz_algebra_int.h"
#include "math/pprz_algebra_float.h"
#include "generated/airframe.h"

/* must be defined by underlying hardware */
extern void imu_impl_init(void);
extern void imu_periodic(void);

struct Imu {
  struct Int32Rates gyro;
  struct Int32Vect3 accel;
  struct Int32Vect3 mag;
  struct Int32Rates gyro_prev;
  struct Int32Vect3 accel_prev;
  struct Int32Rates gyro_neutral;
  struct Int32Vect3 accel_neutral;
  struct Int32Vect3 mag_neutral;
  struct Int32Rates gyro_unscaled;
  struct Int32Vect3 accel_unscaled;
  struct Int32Vect3 mag_unscaled;
  struct Int32Quat  body_to_imu_quat;
  struct Int32RMat  body_to_imu_rmat;
};

/* abstract IMU interface providing floating point interface  */
struct ImuFloat {
  struct FloatRates   gyro;
  struct FloatVect3   accel;
  struct FloatVect3   mag;
  struct FloatRates   gyro_prev;
  struct FloatEulers  body_to_imu_eulers;
  struct FloatQuat    body_to_imu_quat;
  struct FloatRMat    body_to_imu_rmat;
  uint32_t sample_count;
};

extern void imu_float_init(struct ImuFloat* imuf);
extern struct Imu imu;

/* underlying hardware */
#ifdef IMU_TYPE_H
#include IMU_TYPE_H
#endif

extern void imu_init(void);

#define ImuScaleGyro(_imu) {					\
    RATES_COPY(_imu.gyro_prev, _imu.gyro);				\
    _imu.gyro.p = ((_imu.gyro_unscaled.p - _imu.gyro_neutral.p)*IMU_GYRO_P_SIGN*IMU_GYRO_P_SENS_NUM)/IMU_GYRO_P_SENS_DEN; \
    _imu.gyro.q = ((_imu.gyro_unscaled.q - _imu.gyro_neutral.q)*IMU_GYRO_Q_SIGN*IMU_GYRO_Q_SENS_NUM)/IMU_GYRO_Q_SENS_DEN; \
    _imu.gyro.r = ((_imu.gyro_unscaled.r - _imu.gyro_neutral.r)*IMU_GYRO_R_SIGN*IMU_GYRO_R_SENS_NUM)/IMU_GYRO_R_SENS_DEN; \
  }


#define ImuScaleAccel(_imu) {					\
    VECT3_COPY(_imu.accel_prev, _imu.accel);				\
    _imu.accel.x = ((_imu.accel_unscaled.x - _imu.accel_neutral.x)*IMU_ACCEL_X_SIGN*IMU_ACCEL_X_SENS_NUM)/IMU_ACCEL_X_SENS_DEN; \
    _imu.accel.y = ((_imu.accel_unscaled.y - _imu.accel_neutral.y)*IMU_ACCEL_Y_SIGN*IMU_ACCEL_Y_SENS_NUM)/IMU_ACCEL_Y_SENS_DEN; \
    _imu.accel.z = ((_imu.accel_unscaled.z - _imu.accel_neutral.z)*IMU_ACCEL_Z_SIGN*IMU_ACCEL_Z_SENS_NUM)/IMU_ACCEL_Z_SENS_DEN; \
  }

#if defined IMU_MAG_45_HACK
#define ImuScaleMag(_imu) {						\
    int32_t msx = ((_imu.mag_unscaled.x - _imu.mag_neutral.x) * IMU_MAG_X_SIGN * IMU_MAG_X_SENS_NUM) / IMU_MAG_X_SENS_DEN; \
    int32_t msy = ((_imu.mag_unscaled.y - _imu.mag_neutral.y) * IMU_MAG_Y_SIGN * IMU_MAG_Y_SENS_NUM) / IMU_MAG_Y_SENS_DEN; \
    _imu.mag.x = msx - msy;						\
    _imu.mag.y = msx + msy;						\
    _imu.mag.z = ((_imu.mag_unscaled.z - _imu.mag_neutral.z) * IMU_MAG_Z_SIGN * IMU_MAG_Z_SENS_NUM) / IMU_MAG_Z_SENS_DEN; \
  }
#else
#define ImuScaleMag(_imu) {						\
    _imu.mag.x = ((_imu.mag_unscaled.x - _imu.mag_neutral.x) * IMU_MAG_X_SIGN * IMU_MAG_X_SENS_NUM) / IMU_MAG_X_SENS_DEN; \
    _imu.mag.y = ((_imu.mag_unscaled.y - _imu.mag_neutral.y) * IMU_MAG_Y_SIGN * IMU_MAG_Y_SENS_NUM) / IMU_MAG_Y_SENS_DEN; \
    _imu.mag.z = ((_imu.mag_unscaled.z - _imu.mag_neutral.z) * IMU_MAG_Z_SIGN * IMU_MAG_Z_SENS_NUM) / IMU_MAG_Z_SENS_DEN; \
  }
#endif


#endif /* IMU_H */
