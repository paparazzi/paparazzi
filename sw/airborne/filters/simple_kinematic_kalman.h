/*
 * Copyright (C) 2020 Gautier Hattenberger <gautier.hattenberger@enac.fr>
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

/** @file "filters/simple_kinematic_kalman.h"
 * @author Gautier Hattenberger <gautier.hattenberger@enac.fr>
 * @author Antoine Leclerc, Nathan Puch, Pauline Molitor
 *
 * Basic kinematic kalman filter for tag tracking and constant speed
 */

#ifndef SIMPLE_KINEMATIC_KALMAN_H
#define SIMPLE_KINEMATIC_KALMAN_H

#include "std.h"
#include "math/pprz_algebra_float.h"

#define SIMPLE_KINEMATIC_KALMAN_DIM 6

// Speed update types
#define SIMPLE_KINEMATIC_KALMAN_SPEED_HORIZONTAL  1
#define SIMPLE_KINEMATIC_KALMAN_SPEED_VERTICAL    2
#define SIMPLE_KINEMATIC_KALMAN_SPEED_3D          3

/** Kalman structure
 *
 * state vector: X = [ x xd y yd z zd ]'
 * command vector: U = 0 (constant velocity model)
 * dynamic model: basic kinematic model x_k+1 = x_k + xd_k * dt
 * measures: distance between (fixed and known) anchors and UAV
 *
 * */
struct SimpleKinematicKalman {
  float state[SIMPLE_KINEMATIC_KALMAN_DIM];                               ///< state vector
  float P[SIMPLE_KINEMATIC_KALMAN_DIM][SIMPLE_KINEMATIC_KALMAN_DIM];      ///< covariance matrix
  float Q[SIMPLE_KINEMATIC_KALMAN_DIM][SIMPLE_KINEMATIC_KALMAN_DIM];      ///< process noise matrix
  float F[SIMPLE_KINEMATIC_KALMAN_DIM][SIMPLE_KINEMATIC_KALMAN_DIM];      ///< dynamic matrix
  float Hp[SIMPLE_KINEMATIC_KALMAN_DIM / 2][SIMPLE_KINEMATIC_KALMAN_DIM]; ///< observation matrix for position
  float Hs[SIMPLE_KINEMATIC_KALMAN_DIM / 2][SIMPLE_KINEMATIC_KALMAN_DIM]; ///< observation matrix for speed
  float r;                                                                ///< measurement noise (assumed the same for all anchors)
  float dt;                                                               ///< prediction step (in seconds)
};

/** Init SimpleKinematicKalman internal struct
 *
 * @param[in] kalman SimpleKinematicKalman structure
 * @param[in] P0_pos initial covariance on position
 * @param[in] P0_speed initial covariance on speed
 * @param[in] Q_sigma2 process noise
 * @param[in] r measurement noise
 * @param[in] dt prediction time step in seconds
 */
extern void simple_kinematic_kalman_init(struct SimpleKinematicKalman *kalman, float P0_pos, float P0_speed,
    float Q_sigma2, float r, float dt);

/** Set initial state vector
 *
 * This function should be called after initialization of the kalman struct and before
 * running the filter for better results and faster convergence
 *
 * @param[in] kalman SimpleKinematicKalman structure
 * @param[in] pos initial position
 * @param[in] speed initial speed
 */
extern void simple_kinematic_kalman_set_state(struct SimpleKinematicKalman *kalman, struct FloatVect3 pos,
    struct FloatVect3 speed);

/** Get current state
 *
 * @param[in] kalman SimpleKinematicKalman structure
 * @param[out] pos current position
 * @param[out] speed current speed
 */
extern void simple_kinematic_kalman_get_state(struct SimpleKinematicKalman *kalman, struct FloatVect3 *pos,
    struct FloatVect3 *speed);

/** Get current pos
 *
 * @param[in] kalman SimpleKinematicKalman structure
 * @return current position
 */
extern struct FloatVect3 simple_kinematic_kalman_get_pos(struct SimpleKinematicKalman *kalman);

/** Get current speed
 *
 * @param[in] kalman SimpleKinematicKalman structure
 * @return current speed
 */
extern struct FloatVect3 simple_kinematic_kalman_get_speed(struct SimpleKinematicKalman *kalman);

/** Update process and measurement noises
 *
 * @param[in] kalman SimpleKinematicKalman structure
 * @param[in] Q_sigma2 process noise
 * @param[in] r measurement noise
 */
extern void simple_kinematic_kalman_update_noise(struct SimpleKinematicKalman *kalman, float Q_sigma2, float r);

/** Prediction step
 *
 * @param[in] kalman SimpleKinematicKalman structure
 */
extern void simple_kinematic_kalman_predict(struct SimpleKinematicKalman *kalman);

/** Update step based on each new distance data
 *
 * @param[in] kalman SimpleKinematicKalman structure
 * @param[in] pos position of the target from which the distance is measured
 */
extern void simple_kinematic_kalman_update_pos(struct SimpleKinematicKalman *kalman, struct FloatVect3 pos);

/** Update step based on speed measure
 *
 * @param[in] kalman SimpleKinematicKalman structure
 * @param[in] speed new speed measurement
 * @param[in] type 1: horizontal ground speed, 2: vertical ground speed, 3: 3D ground speed
 */
extern void simple_kinematic_kalman_update_speed(struct SimpleKinematicKalman *kalman, struct FloatVect3 speed, uint8_t type);

#endif
