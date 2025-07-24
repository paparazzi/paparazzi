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

#include "std.h"
#include "generated/airframe.h"
#include <stdio.h>

enum ins_ext_pose_state {
  EKF_X_POS_X,
  EKF_X_POS_Y,
  EKF_X_POS_Z,
  EKF_X_VEL_X,
  EKF_X_VEL_Y,
  EKF_X_VEL_Z,
  EKF_X_PHI,
  EKF_X_THETA,
  EKF_X_PSI,
  EKF_X_A_BIAS_X,
  EKF_X_A_BIAS_Y,
  EKF_X_A_BIAS_Z,
  EKF_X_G_BIAS_P,
  EKF_X_G_BIAS_Q,
  EKF_X_G_BIAS_R,
  EKF_NUM_STATES
};

enum ins_ext_pose_inputs {
  EKF_U_ACC_X,
  EKF_U_ACC_Y,
  EKF_U_ACC_Z,
  EKF_U_GYRO_P,
  EKF_U_GYRO_Q,
  EKF_U_GYRO_R,
  EKF_NUM_INPUTS
};

enum ins_ext_pose_outputs {
  EKF_Z_POS_X,
  EKF_Z_POS_Y,
  EKF_Z_POS_Z,
  EKF_Z_PHI,
  EKF_Z_THETA,
  EKF_Z_PSI,
  EKF_NUM_OUTPUTS
};

extern float ekf_X[EKF_NUM_STATES];

extern void ins_ext_pose_init(void);
extern void ins_ext_pose_run(void);

extern void ins_ext_pose_msg_update(uint8_t *buf);

#ifdef INS_EXT_VISION_ROTATION
extern struct FloatQuat ins_ext_vision_rot;
#endif

// Logging
extern void ins_ext_pose_log_header(FILE *file);
extern void ins_ext_pose_log_data(FILE *file);

#endif
