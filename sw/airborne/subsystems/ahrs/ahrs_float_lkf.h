/*
 * $Id:  $
 *
 * Copyright (C) 2009 Felix Ruess <felix.ruess@gmail.com>
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

#ifndef AHRS_FLOAT_LKF_H
#define AHRS_FLOAT_LKF_H

#include "subsystems/ahrs.h"
#include "std.h"
#include "math/pprz_algebra_int.h"

extern struct FloatQuat   bafl_quat;
extern struct FloatRates  bafl_bias;
extern struct FloatRates  bafl_rates;
extern struct FloatEulers bafl_eulers;
extern struct FloatRMat   bafl_dcm;

extern struct FloatQuat   bafl_q_a_err;
extern struct FloatQuat   bafl_q_m_err;
extern struct FloatRates  bafl_b_a_err;
extern struct FloatRates  bafl_b_m_err;
extern float bafl_qnorm;
extern float bafl_phi_accel;
extern float bafl_theta_accel;
extern struct FloatVect3  bafl_accel_measure;
extern struct FloatVect3  bafl_mag;

#define BAFL_SSIZE 6
extern float bafl_P[BAFL_SSIZE][BAFL_SSIZE];
extern float bafl_X[BAFL_SSIZE];

extern float bafl_sigma_accel;
extern float bafl_sigma_mag;
extern float bafl_R_accel;
extern float bafl_R_mag;

extern float bafl_Q_att;
extern float bafl_Q_gyro;


#define ahrs_float_lkf_SetRaccel(_v) { \
  bafl_sigma_accel = _v; \
  bafl_R_accel = _v * _v; \
}
#define ahrs_float_lkf_SetRmag(_v) { \
  bafl_sigma_mag = _v; \
  bafl_R_mag = _v * _v; \
}

#endif /* AHRS_FLOAT_LKF_H */

