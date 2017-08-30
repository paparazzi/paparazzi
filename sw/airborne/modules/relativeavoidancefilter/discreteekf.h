/*
 * Copyright (C) Mario Coppola
 *
 * This file is part of paparazzi
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
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */
/**
 * @file "modules/relativeavoidancefilter/discreteekf.h"
 * @author Mario Coppola
 * Discrete Extended Kalman Filter for Relative Localization
 */

#ifndef DISCRETEEKF_H
#define DISCRETEEKF_H

#include "fmatrix.h"
#include "stdlib.h"
#include "string.h"
#include "math.h"

#define EKF_N 7
#define EKF_M 6

typedef struct ekf_filter {

  /* state                           */
  float X[EKF_N];
  /* state prediction                */
  float Xp[EKF_N];
  /* measurement prediction          */
  float Zp[EKF_M];
  /* state covariance matrix         */
  float P[EKF_N*EKF_N];
  /* process covariance noise        */
  float Q[EKF_N*EKF_N];
  /* measurement covariance noise    */
  float R[EKF_M*EKF_M];
  /* jacobian of the measure wrt X   */
  float H[EKF_N*EKF_M];

  /* Temp matrices */
  float tmp1[EKF_N*EKF_N];
  float tmp2[EKF_N*EKF_N];
  float tmp3[EKF_N*EKF_N];

  float dt;

} ekf_filter;

typedef struct btmodel {
  float Pn;
  float gammal;
} btmodel;

/*
 * Basic functions describing evolution and measure
 */

extern void linear_filter(float* X, float dt, float *dX, float* A);
extern void linear_measure(float*X, float* Y, float *H, btmodel *model);

extern void ekf_filter_new(ekf_filter* filter);

extern void ekf_filter_setup(
					ekf_filter *filter, 
					float* Q,
					float* R,
          float t);

extern void ekf_filter_reset(ekf_filter *filter);

extern void ekf_filter_predict(ekf_filter *filter, btmodel *model);

extern void ekf_filter_update(ekf_filter *filter, float *y);

extern void ekf_filter_get_state(ekf_filter* filter, float *X, float* P);

#endif /* EKF_H */