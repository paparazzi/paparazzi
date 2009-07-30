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

#ifndef BOOZ_AHRS_FLOAT_LKF_H
#define BOOZ_AHRS_FLOAT_LKF_H

#include "booz_ahrs.h"
#include "std.h"
#include "math/pprz_algebra_int.h"

extern struct FloatQuat   bafl_quat;
extern struct FloatRates  bafl_bias;
extern struct FloatRates  bafl_rates;
extern struct FloatEulers bafl_eulers;
extern struct FloatRMat   bafl_dcm;

extern struct FloatQuat   bafl_quat_err;
extern struct FloatRates  bafl_bias_err;

#define BAFL_SSIZE 6
extern float bafl_P[BAFL_SSIZE][BAFL_SSIZE];
extern float bafl_X[BAFL_SSIZE];

extern float bafl_sigma_accel;
extern float bafl_sigma_mag;
extern float bafl_R_accel;
extern float bafl_R_mag;


#define booz_ahrs_float_lkf_SetRaccel(_v) { \
  bafl_sigma_accel = _v; \
  bafl_R_accel = _v * _v; \
}
#define booz_ahrs_float_lkf_SetRmag(_v) { \
  bafl_sigma_mag = _v; \
  bafl_R_mag = _v * _v; \
}

#endif /* BOOZ_AHRS_FLOAT_LKF_H */

