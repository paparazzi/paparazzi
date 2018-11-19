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
 * @file "modules/relative_localization_filter/discrete_ekf.h"
 * @author Mario Coppola
 * Discrete Extended Kalman Filter for Relative Localization
 */

#ifndef DISCRETE_EKF_H
#define DISCRETE_EKF_H

#include "stdlib.h"
#include "string.h"
#include "math.h"

#define EKF_N 7
#define EKF_M 6

struct discrete_ekf {
  float X[EKF_N];  // state X
  float Xp[EKF_N]; // state prediction
  float Zp[EKF_M]; // measurement prediction
  float P[EKF_N][EKF_N]; // state covariance matrix
  float Q[EKF_N][EKF_N]; // proces covariance noise
  float R[EKF_M][EKF_M]; // measurement covariance noise
  float H[EKF_M][EKF_N]; // jacobian of the measure wrt X
  float Ht[EKF_N][EKF_M]; // transpose of H

  float tmp1[EKF_N][EKF_N];
  float tmp2[EKF_N][EKF_N];
  float tmp3[EKF_N][EKF_N];

  float dt;
};

extern void linear_filter(float *X, float dt, float *dX, float **A);
extern void linear_measure(float *X, float *Y, float **H);

extern void discrete_ekf_new(struct discrete_ekf *filter);
extern void discrete_ekf_predict(struct discrete_ekf *filter);
extern void discrete_ekf_update(struct discrete_ekf *filter, float *y);

#endif /* DISCRETE_EKF_H */
