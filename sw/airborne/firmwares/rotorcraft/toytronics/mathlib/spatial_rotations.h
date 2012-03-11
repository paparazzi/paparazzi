/*
 * This file is part of mathlib.
 *
 * Copyright (C) 2010-2011 Greg Horn <ghorn@stanford.edu>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

/*
 * spatial_rotations.h
 * Math library spatial rotation related operations.
 */

#ifndef __SPATIAL_ROTATIONS_H__
#define __SPATIAL_ROTATIONS_H__

#include "quat.h"
#include "xyz.h"

#ifndef EULER_T
#define EULER_T
typedef struct euler_t {
  double roll;
  double pitch;
  double yaw;
} euler_t;
#endif // EULER_T

#ifdef __cplusplus
extern "C"{
#endif

  void rotate_xyz_about_x( xyz_t * b, const xyz_t * const a, const double rot_angle);
  void euler321_of_quat(euler_t *e, const quat_t * const q);
  void euler321_of_dcm(euler_t *e, const double * const R);
  void quat_of_dcm_a2b(quat_t * q, const double * const R);
  void quat_of_dcm_b2a(quat_t * q, const double * const R);
  void quat_of_euler321(quat_t * q, const euler_t * const e);
  void dcm_of_quat_a2b(double *R_a2b, const quat_t * const q_a2b);
  void dcm_of_quat_b2a(double *R_b2a, const quat_t * const q_a2b);
  void rot_vec_by_dcm_a2b(xyz_t *vec_b, const double * const R_a2b, const xyz_t * const vec_a);
  void rot_vec_by_dcm_b2a(xyz_t *vec_a, const double * const R_a2b, const xyz_t * const vec_b);
  void rot_vec_by_quat_a2b(xyz_t *vec_b, const quat_t * const q_a2b, const xyz_t * const vec_a);
  void rot_vec_by_quat_b2a(xyz_t *vec_a, const quat_t * const q_a2b, const xyz_t * const vec_b);
  void get_wind_angles_from_v_bw_b(double * alpha, double * beta, double * airspeed, const xyz_t * const v_bw_b);
  void get_wind_angles( double * alpha,
                        double * beta,
                        double * airspeed,
                        xyz_t * v_bw_b_out,
                        const quat_t * const q_n2b,
                        const xyz_t * const v_bn_b,
                        const xyz_t * const v_wn_n);
  void v_bw_b_from_wind_angles( xyz_t * v_bw_b, const double alpha, const double beta, const double airspeed);

#ifdef __cplusplus
}
#endif

#endif // __SPATIAL_ROTATIONS_H__
