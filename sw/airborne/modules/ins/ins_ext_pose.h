/*
 * Copyright (C) 2023 MAVLab
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

/**
 * @file modules/ins/ins_ext_pose.h
 * Integrated Navigation System interface.
 */

/*
Extened kalman filter based on https://en.wikipedia.org/wiki/Extended_Kalman_filter
Section 5.3: Non-additive noise formulation and equations
*/

#ifndef INS_EXT_POSE_H
#define INS_EXT_POSE_H

#define EKF_NUM_STATES 15
#define EKF_NUM_INPUTS 6
#define EKF_NUM_OUTPUTS 6

#include "std.h" 

#include <stdio.h>


extern float ekf_X[EKF_NUM_STATES];

void ekf_set_diag(float **a, float *b, int n);

extern void ekf_init(void);

void ekf_f(const float X[EKF_NUM_STATES], const float U[EKF_NUM_INPUTS], float out[EKF_NUM_STATES]);
void ekf_F(const float X[EKF_NUM_STATES], const float U[EKF_NUM_INPUTS], float out[EKF_NUM_STATES][EKF_NUM_STATES]);
void ekf_L(const float X[EKF_NUM_STATES], const float U[EKF_NUM_INPUTS], float out[EKF_NUM_STATES][EKF_NUM_INPUTS]);

void ekf_f_rk4(const float X[EKF_NUM_STATES], const float U[EKF_NUM_INPUTS], const float dt, float out[EKF_NUM_STATES]);

extern void ekf_step(const float U[EKF_NUM_INPUTS], const float Z[EKF_NUM_OUTPUTS], const float dt);
extern void ekf_prediction_step(const float U[EKF_NUM_INPUTS], const float dt);
extern void ekf_measurement_step(const float Z[EKF_NUM_OUTPUTS]);

extern void ekf_run(void);

extern void external_pose_update(uint8_t *buf);

// Logging
extern void ins_ext_pos_log_header(FILE *file);
extern void ins_ext_pos_log_data(FILE *file);


#endif
