/*
 * Copyright (C) 2008-2010 The Paparazzi Team
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

#ifndef AHRS_H
#define AHRS_H

#include "std.h"
#include "math/pprz_algebra_int.h"
#include "math/pprz_algebra_float.h"

#define AHRS_UNINIT  0
#define AHRS_RUNNING 1

/* underlying includes (needed for parameters) */
#ifdef AHRS_TYPE_H
#include AHRS_TYPE_H
#endif


struct Ahrs {

    struct Int32Quat   ltp_to_imu_quat;
    struct Int32Eulers ltp_to_imu_euler;
    struct Int32RMat   ltp_to_imu_rmat;
    struct Int32Rates  imu_rate;

    struct Int32Quat   ltp_to_body_quat;
    struct Int32Eulers ltp_to_body_euler;
    struct Int32RMat   ltp_to_body_rmat;
    struct Int32Rates  body_rate;

    uint8_t status;
};

struct AhrsFloat {
  struct FloatQuat   ltp_to_imu_quat;
  struct FloatEulers ltp_to_imu_euler;
  struct FloatRMat   ltp_to_imu_rmat;
  struct FloatRates  imu_rate;
  struct FloatRates  imu_rate_previous;
  struct FloatRates  imu_rate_d;

  struct FloatQuat   ltp_to_body_quat;
  struct FloatEulers ltp_to_body_euler;
  struct FloatRMat   ltp_to_body_rmat;
  struct FloatRates  body_rate;
  struct FloatRates  body_rate_d;

  uint8_t status;
};

extern struct Ahrs ahrs;
extern struct AhrsFloat ahrs_float;

extern float ahrs_mag_offset;

#define AHRS_FLOAT_OF_INT32() {						       \
    QUAT_FLOAT_OF_BFP(ahrs_float.ltp_to_body_quat, ahrs.ltp_to_body_quat);     \
    EULERS_FLOAT_OF_BFP(ahrs_float.ltp_to_body_euler, ahrs.ltp_to_body_euler); \
    RATES_FLOAT_OF_BFP(ahrs_float.body_rate, ahrs.body_rate);		       \
  }

#define AHRS_INT_OF_FLOAT() {                                                  \
    QUAT_BFP_OF_REAL(ahrs.ltp_to_body_quat, ahrs_float.ltp_to_body_quat);      \
    EULERS_BFP_OF_REAL(ahrs.ltp_to_body_euler, ahrs_float.ltp_to_body_euler);  \
    RMAT_BFP_OF_REAL(ahrs.ltp_to_body_rmat, ahrs_float.ltp_to_body_rmat);      \
    RATES_BFP_OF_REAL(ahrs.body_rate, ahrs_float.body_rate);                   \
  }

extern void ahrs_init(void);
extern void ahrs_align(void);
extern void ahrs_propagate(void);
extern void ahrs_update_accel(void);
extern void ahrs_update_mag(void);

#endif /* AHRS_H */
