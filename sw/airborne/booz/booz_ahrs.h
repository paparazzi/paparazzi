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

#ifndef BOOZ_AHRS_H
#define BOOZ_AHRS_H

#include "std.h"
#include "math/pprz_algebra_int.h"
#include "math/pprz_algebra_float.h"
#include "ahrs/booz_ahrs_aligner.h"

#define BOOZ_AHRS_UNINIT  0
#define BOOZ_AHRS_RUNNING 1

struct BoozAhrs {
    struct Int32Eulers ltp_to_body_euler;
    struct Int32Quat ltp_to_body_quat;
    struct Int32RMat ltp_to_body_rmat;
    struct Int32Eulers ltp_to_imu_euler;
    struct Int32Quat ltp_to_imu_quat;
    struct Int32RMat ltp_to_imu_rmat;
    struct Int32Rates imu_rate;
    struct Int32Rates body_rate;
    uint8_t status;
};

struct BoozAhrsFloat {
  struct FloatQuat   ltp_to_body_quat;
  struct FloatEulers ltp_to_body_euler;
  struct FloatRates  body_rate;  
  uint8_t status;
};

extern struct BoozAhrs booz_ahrs;
extern struct BoozAhrsFloat booz_ahrs_float;
extern struct Int32Vect3 booz_ahrs_accel_mean;

#define BOOZ_AHRS_FLOAT_OF_INT32() {					\
    QUAT_FLOAT_OF_BFP(booz_ahrs_float.ltp_to_body_quat, booz_ahrs.ltp_to_body_quat); \
    RATES_FLOAT_OF_BFP(booz_ahrs_float.body_rate, booz_ahrs.body_rate); \
    booz_ahrs_float.ltp_to_body_euler.phi = ANGLE_FLOAT_OF_BFP(booz_ahrs.ltp_to_body_euler.phi); \
    booz_ahrs_float.ltp_to_body_euler.theta = ANGLE_FLOAT_OF_BFP(booz_ahrs.ltp_to_body_euler.theta); \
    booz_ahrs_float.ltp_to_body_euler.psi = ANGLE_FLOAT_OF_BFP(booz_ahrs.ltp_to_body_euler.psi); \
  }

extern void booz_ahrs_init(void);
extern void booz_ahrs_align(void);
extern void booz_ahrs_propagate(void);
extern void booz_ahrs_update(void);
extern void booz_ahrs_update_accel(void);
extern void booz_ahrs_update_mag(void);

extern void booz_ahrs_init_accel_rb(void);
extern void booz_ahrs_store_accel(void);
extern void booz_ahrs_compute_accel_mean(uint8_t n);

#endif /* BOOZ_AHRS_H */
