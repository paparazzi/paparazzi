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
 * @file "modules/relative_localization_filter/discrete_ekf_no_north.h"
 * @author Steven van der Helm, Mario Coppola
 * Discrete Extended Kalman Filter for Relative Localization
 * This implementation is was used in
 * van der Helm et al. "On-board Range-based Relative Localization for Micro Aerial Vehicles in indoor Leader-Follower Flight." (2018).
 * Available at https://arxiv.org/pdf/1805.07171.pdf
 */

#ifndef DISCRETE_EKF_NO_NORTH_H
#define DISCRETE_EKF_NO_NORTH_H

#include "stdlib.h"
#include "string.h"
#include "math.h"

#define EKF_N 9
#define EKF_M 7
#define EKF_L 6

struct discrete_ekf_no_north {
  float X[EKF_N];  // state X
  float Xp[EKF_N]; // state prediction
  float Zp[EKF_M]; // measurement prediction
  float P[EKF_N][EKF_N]; // state covariance matrix
  float Q[EKF_N][EKF_N]; // proces covariance noise
  float R[EKF_M][EKF_M]; // measurement covariance noise
  float H[EKF_M][EKF_N]; // jacobian of the measure wrt X
  float G[EKF_N][EKF_L]; // Noise input
  float Ht[EKF_N][EKF_M]; // transpose of H
  float Phi[EKF_N][EKF_N]; // Jacobian
  float Gamma[EKF_N][EKF_L]; // Noise input
  float Fx[EKF_N][EKF_N]; // Jacobian of state

  float tmp1[EKF_N][EKF_N];
  float tmp2[EKF_N][EKF_N];
  float tmp3[EKF_N][EKF_N];
  float tmp4[EKF_N][EKF_N];

  float dt;
};


void extractPhiGamma(float **inmat, float **phi, float **gamma, int m, int n_a, int n_b);
void float_mat_combine(float **a, float **b, float **o, int m, int n_a, int n_b);
void c2d(int m, int nA, int nB, float **Fx, float **G, float dt, float **phi, float **gamma);
void discrete_ekf_no_north_fsym(float *statein, float *input, float *output);
void discrete_ekf_no_north_hsym(float *statein, float *output);
void discrete_ekf_no_north_Fx(float *statein, float *input, float **output);
void discrete_ekf_no_north_G(float *statein, float **output);
void discrete_ekf_no_north_Hx(float *statein, float **output);

extern void discrete_ekf_no_north_new(struct discrete_ekf_no_north *filter);
extern void discrete_ekf_no_north_predict(struct discrete_ekf_no_north *filter, float *U);
extern void discrete_ekf_no_north_update(struct discrete_ekf_no_north *filter, float *y);

#endif /* DISCRETE_EKF_NO_NORTH_H */
