/*
 * Copyright (C) 2021 Gautier Hattenberger <gautier.hattenberger@enac.fr>
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
 * @file "filters/linear_kalman_filter.c"
 *
 * Generic discrete Linear Kalman Filter
 */

#ifndef LINEAR_KALMAN_FILTER_H
#define LINEAR_KALMAN_FILTER_H

#include "std.h"

// maximum size for the state vector
#ifndef KF_MAX_STATE_SIZE
#define KF_MAX_STATE_SIZE 6
#endif

// maximum size for the command vector
#ifndef KF_MAX_CMD_SIZE
#define KF_MAX_CMD_SIZE 2
#endif

// maximum size for the measurement vector
#ifndef KF_MAX_MEAS_SIZE
#define KF_MAX_MEAS_SIZE 6
#endif

struct linear_kalman_filter {
  // filled by user after calling init function
  float A[KF_MAX_STATE_SIZE][KF_MAX_STATE_SIZE];  ///< dynamic matrix
  float B[KF_MAX_STATE_SIZE][KF_MAX_CMD_SIZE];    ///< command matrix
  float C[KF_MAX_MEAS_SIZE][KF_MAX_STATE_SIZE];   ///< observation matrix
  float P[KF_MAX_STATE_SIZE][KF_MAX_STATE_SIZE];  ///< state covariance matrix
  float Q[KF_MAX_STATE_SIZE][KF_MAX_STATE_SIZE];  ///< proces covariance noise
  float R[KF_MAX_MEAS_SIZE][KF_MAX_MEAS_SIZE];    ///< measurement covariance noise

  float X[KF_MAX_STATE_SIZE];                     ///< estimated state X

  uint8_t n;  ///< state vector size (<= KF_MAX_STATE_SIZE)
  uint8_t c;  ///< command vector size (<= KF_MAX_CMD_SIZE)
  uint8_t m;  ///< measurement vector size (<= KF_MAX_MEAS_SIZE)
};

/** Init all matrix and vectors to zero
 *
 * @param filter pointer to a filter structure
 * @param n size of the state vector
 * @param c size of the command vector
 * @param m size of the measurement vector
 * @return false if n, c or m are larger than the maximum value
 */
extern bool linear_kalman_filter_init(struct linear_kalman_filter *filter, uint8_t n, uint8_t c, uint8_t m);

/** Prediction step
 *
 * X = Ad * X + Bd * U
 * P = Ad * P * Ad' + Q
 *
 * @param filter pointer to the filter structure
 * @param U command vector
 */
extern void linear_kalman_filter_predict(struct linear_kalman_filter *filter, float *U);

/** Update step
 *
 * S = Cd * P * Cd' + R
 * K = P * Cd' / S
 * X = X + K * (Y - Cd * X)
 * P = P - K * Cd * P
 *
 * @param filter pointer to the filter structure
 * @param Y measurement vector
 */
extern void linear_kalman_filter_update(struct linear_kalman_filter *filter, float *Y);

#endif /* DISCRETE_EKF_H */
