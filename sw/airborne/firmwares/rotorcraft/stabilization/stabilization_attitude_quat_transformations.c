/*
 * Copyright (C) 2013 Felix Ruess <felix.ruess@gmail.com>
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

/** @file stabilization_attitude_quat_transformations.c
 *  Quaternion transformation functions.
 */

#include "stabilization_attitude_quat_transformations.h"

void quat_from_rpy_cmd_i(struct Int32Quat *quat, struct Int32Eulers *rpy)
{
  struct FloatEulers rpy_f;
  EULERS_FLOAT_OF_BFP(rpy_f, *rpy);
  struct FloatQuat quat_f;
  quat_from_rpy_cmd_f(&quat_f, &rpy_f);
  QUAT_BFP_OF_REAL(*quat, quat_f);
}

void quat_from_rpy_cmd_f(struct FloatQuat *quat, struct FloatEulers *rpy)
{
  // only a plug for now... doesn't apply roll/pitch wrt. current yaw angle

  /* orientation vector describing simultaneous rotation of roll/pitch/yaw */
  const struct FloatVect3 ov = {rpy->phi, rpy->theta, rpy->psi};
  /* quaternion from that orientation vector */
  float_quat_of_orientation_vect(quat, &ov);

}

void quat_from_earth_cmd_i(struct Int32Quat *quat, struct Int32Vect2 *cmd, int32_t heading)
{
  // use float conversion for now...
  struct FloatVect2 cmd_f;
  cmd_f.x = ANGLE_FLOAT_OF_BFP(cmd->x);
  cmd_f.y = ANGLE_FLOAT_OF_BFP(cmd->y);
  float heading_f = ANGLE_FLOAT_OF_BFP(heading);

  struct FloatQuat quat_f;
  quat_from_earth_cmd_f(&quat_f, &cmd_f, heading_f);

  // convert back to fixed point
  QUAT_BFP_OF_REAL(*quat, quat_f);
}

void quat_from_earth_cmd_f(struct FloatQuat *quat, struct FloatVect2 *cmd, float heading)
{

  /* cmd_x is positive to north = negative pitch
   * cmd_y is positive to east = positive roll
   *
   * orientation vector describing simultaneous rotation of roll/pitch
   */
  const struct FloatVect3 ov = {cmd->y, -cmd->x, 0.0};
  /* quaternion from that orientation vector */
  struct FloatQuat q_rp;
  float_quat_of_orientation_vect(&q_rp, &ov);

  /* as rotation matrix */
  struct FloatRMat R_rp;
  float_rmat_of_quat(&R_rp, &q_rp);
  /* body x-axis (before heading command) is first row */
  struct FloatVect3 b_x;
  VECT3_ASSIGN(b_x, R_rp.m[0], R_rp.m[1], R_rp.m[2]);
  /* body z-axis (thrust vect) is last row */
  struct FloatVect3 thrust_vect;
  VECT3_ASSIGN(thrust_vect, R_rp.m[6], R_rp.m[7], R_rp.m[8]);

  /*
   * Instead of using the psi setpoint angle to rotate around the body z-axis,
   * calculate the real angle needed to align the projection of the body x-axis
   * onto the horizontal plane with the psi setpoint.
   *
   * angle between two vectors a and b:
   * angle = atan2(norm(cross(a,b)), dot(a,b)) * sign(dot(cross(a,b), n))
   * where the normal n is the thrust vector (i.e. both a and b lie in that plane)
   */

  // desired heading vect in earth x-y plane
  const struct FloatVect3 psi_vect = {cosf(heading), sinf(heading), 0.0};

  /* projection of desired heading onto body x-y plane
   * dot(b,n) = 0
   */
  float dot = VECT3_DOT_PRODUCT(psi_vect, thrust_vect);

  struct FloatVect3 b;
  VECT3_ASSIGN(b, psi_vect.x, psi_vect.y, -dot/thrust_vect.z);

  dot = VECT3_DOT_PRODUCT(b_x, b);
  struct FloatVect3 cross;
  VECT3_CROSS_PRODUCT(cross, b_x, b);
  // norm of the cross product
  float nc = FLOAT_VECT3_NORM(cross);

  /* Compute the half-angle values directly from the two vectors. */
  float n = sqrtf(nc * nc + dot * dot);
  float cos_yaw2;
  float sin_yaw2;
  if (n == 0.0f) {
    // Same zero-angle result as atan2(0, 0) in the original code.
    cos_yaw2 = 1.0f;
    sin_yaw2 = 0.0f;
  } else {
    float c = dot / n;
    // Protect the square roots from small floating-point overshoots.
    c = c > 1.0f ? 1.0f : c;
    c = c < -1.0f ? -1.0f : c;
    cos_yaw2 = sqrtf(0.5f * (1.0f + c));
    sin_yaw2 = sqrtf(0.5f * (1.0f - c));

    // Preserve the sign of the rotation around the thrust axis.
    float dot_cross_ab = VECT3_DOT_PRODUCT(cross, thrust_vect);
    if (dot_cross_ab < 0) {
      sin_yaw2 = -sin_yaw2;
    }
  }

  /* quaternion with yaw command */
  struct FloatQuat q_yaw;
  QUAT_ASSIGN(q_yaw, cos_yaw2, 0.0, 0.0, sin_yaw2);

  /* final setpoint: apply roll/pitch, then yaw around resulting body z-axis */
  float_quat_comp(quat, &q_rp, &q_yaw);
  float_quat_normalize(quat);
  float_quat_wrap_shortest(quat);

}
