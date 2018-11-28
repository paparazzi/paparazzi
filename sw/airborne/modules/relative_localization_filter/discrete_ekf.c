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
 * @file "modules/relative_localization_filter/discrete_ekf.c"
 * @author Mario Coppola
 * Discrete Extended Kalman Filter for Relative Localization
 */

#include "modules/relative_localization_filter/discrete_ekf.h"
#include "math/pprz_algebra_float.h"
#include <math.h>
#include <stdio.h> // needed for the printf statements

// Weights are based on: Coppola et al, "On-board Communication-based Relative Localization for Collision Avoidance in Micro Air Vehicle teams", 2017
void discrete_ekf_new(struct discrete_ekf *filter)
{
  // P Matrix
  MAKE_MATRIX_PTR(_P, filter->P, EKF_N);
  float_mat_diagonal_scal(_P, 1.f, EKF_N);
  filter->P[2][2] = 0.1;
  filter->P[3][3] = 0.1;
  filter->P[4][4] = 0.1;
  filter->P[5][5] = 0.1;
  filter->P[6][6] = 0.1;

  // Q Matrix
  MAKE_MATRIX_PTR(_Q, filter->Q, EKF_N);
  float_mat_diagonal_scal(_Q, powf(0.3, 2.f), EKF_N);
  filter->Q[0][0] = 0.01;
  filter->Q[1][1] = 0.01;

  MAKE_MATRIX_PTR(_R, filter->R, EKF_M);
  float_mat_diagonal_scal(_R, powf(0.1, 2.f), EKF_M);
  filter->R[0][0] = 0.2;

  // Initial assumptions
  float_vect_zero(filter->X, EKF_N);
  filter->X[0] = 1.0; // filter->X[0] and/or filter->[1] cannot be = 0
  filter->X[1] = 1.0; // filter->X[0] and/or filter->[1] cannot be = 0 
  filter->dt = 0.1;
}

/* Perform the prediction step

    Predict state
      x_p = f(x);
      A = Jacobian of f(x)

    Predict P
      P = A * P * A' + Q;

    Predict measure
      z_p = h(x_p)
      H = Jacobian of h(x)

*/
void discrete_ekf_predict(struct discrete_ekf *filter)
{
  float dX[EKF_N];

  MAKE_MATRIX_PTR(_tmp1, filter->tmp1, EKF_N);
  MAKE_MATRIX_PTR(_tmp2, filter->tmp2, EKF_N);
  MAKE_MATRIX_PTR(_tmp3, filter->tmp3, EKF_N);
  MAKE_MATRIX_PTR(_H,    filter->H,    EKF_N);
  MAKE_MATRIX_PTR(_P,    filter->P,    EKF_N);
  MAKE_MATRIX_PTR(_Q,    filter->Q,    EKF_N);

  // Fetch dX and A given X and dt and input u
  linear_filter(filter->X, filter->dt, dX, _tmp1); // (note: _tmp1 = A)

  // Get state prediction Xp = X + dX
  float_vect_sum(filter->Xp, filter->X, dX, EKF_N);

  // Get measurement prediction (Zp) and Jacobian (H)
  linear_measure(filter->Xp, filter->Zp, _H);

  // P = A * P * A' + Q
  float_mat_mul(_tmp2, _tmp1, _P, EKF_N, EKF_N, EKF_N); // tmp2(=A*P) = A(=_tmp1)*_P
  float_mat_transpose_square(_tmp1, EKF_N); // tmp1 = A'
  float_mat_mul(_tmp3, _tmp2, _tmp1, EKF_N, EKF_N, EKF_N); // tmp3 = tmp2*tmp1 = A*P * A'
  float_mat_sum(_P, _tmp3, _Q, EKF_N, EKF_N); // P = tmp3(=A*P*A') + Q
}

/* Perform the update step

    Get Kalman Gain
      P12 = P * H';
      K = P12/(H * P12 + R);

    Update x
      x = x_p + K * (z - z_p);

    Update P
      P = (eye(numel(x)) - K * H) * P;
*/
void discrete_ekf_update(struct discrete_ekf *filter, float *Z)
{
  MAKE_MATRIX_PTR(_tmp1, filter->tmp1, EKF_N);
  MAKE_MATRIX_PTR(_tmp2, filter->tmp2, EKF_N);
  MAKE_MATRIX_PTR(_tmp3, filter->tmp3, EKF_N);
  MAKE_MATRIX_PTR(_P,    filter->P,    EKF_N);
  MAKE_MATRIX_PTR(_H,    filter->H,    EKF_M);
  MAKE_MATRIX_PTR(_Ht,   filter->Ht,   EKF_N);
  MAKE_MATRIX_PTR(_R,    filter->R,    EKF_M);

  //  E = H * P * H' + R
  float_mat_transpose(_Ht, _H, EKF_M, EKF_N); // Ht = H'
  float_mat_mul(_tmp2, _P, _Ht, EKF_N, EKF_N, EKF_M); // tmp2 = P*Ht = P*H'
  float_mat_mul(_tmp1, _H, _tmp2, EKF_M, EKF_N, EKF_M); // tmp1 = H*P*H'
  float_mat_sum(_tmp3, _tmp1, _R, EKF_M, EKF_M); // E = tmp1(=H*P*H') + R

  // K = P * H' * inv(E)
  float_mat_invert(_tmp1, _tmp3, EKF_M); // tmp1 = inv(E)
  float_mat_mul(_tmp3, _tmp2, _tmp1, EKF_N, EKF_M, EKF_M); // K(tmp3) = tmp2*tmp1

  // P = P - K * H * P
  float_mat_mul(_tmp1, _tmp3, _H, EKF_N, EKF_M, EKF_N); // tmp1 = K*H
  float_mat_mul(_tmp2, _tmp1, _P, EKF_N, EKF_N, EKF_N); // tmp3 = K*H*P
  float_mat_diff(_P, _P, _tmp2, EKF_N, EKF_N); // P - K*H*P

  //  X = X + K * err
  float err[EKF_M];
  float dx_err[EKF_N];

  float_vect_diff(err, Z, filter->Zp, EKF_M); // err = Z - Zp
  float_mat_vect_mul(dx_err, _tmp3, err, EKF_N, EKF_M); // dx_err = K*err
  float_vect_sum(filter->X, filter->Xp, dx_err, EKF_N); // X = Xp + dx_err
}

/* Linearized (Jacobian) filter function */
void linear_filter(float *X, float dt, float *dX, float **A)
{
  // dX
  float_vect_zero(dX, EKF_N);
  dX[0] = -(X[2] - X[4]) * dt;
  dX[1] = -(X[3] - X[5]) * dt;

  // A(x)
  float_mat_diagonal_scal(A, 1.f, EKF_N);
  A[0][2] = -dt;
  A[0][4] =  dt;
  A[1][3] = -dt;
  A[1][5] =  dt;
};

/* Linearized (Jacobian) measure function */
void linear_measure(float *X, float *Z, float **H)
{
  uint8_t row, col;
  Z[0] = sqrt(pow(X[0], 2.f) + pow(X[1], 2.f) + pow(X[6], 2.f));
  Z[1] = X[2]; // x velocity of i (north)
  Z[2] = X[3]; // y velocity of i (east)
  Z[3] = X[4]; // x velocity of j (north)
  Z[4] = X[5]; // y velocity of j (east)
  Z[5] = X[6]; // Height difference

  // Generate the Jacobian Matrix
  for (row = 0 ; row < EKF_M ; row++) {
    for (col = 0 ; col < EKF_N ; col++) {
      // x, y, and z pos columns are affected by the range measurement
      if ((row == 0) && (col == 0 || col == 1 || col == 6)) {
        H[row][col] = X[col] / sqrt(pow(X[0], 2.f) + pow(X[1], 2.f) + pow(X[6], 2.f));
      }

      // All other values are 1
      else if (((row == 1) && (col == 2)) ||
               ((row == 2) && (col == 3)) ||
               ((row == 3) && (col == 4)) ||
               ((row == 4) && (col == 5)) ||
               ((row == 5) && (col == 6))) {
        H[row][col] = 1.f;
      }

      else {
        H[row][col] = 0.f;
      }
    }
  }

};
