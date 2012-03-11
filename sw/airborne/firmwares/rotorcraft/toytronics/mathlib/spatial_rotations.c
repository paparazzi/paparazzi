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
 * spatial_rotations.c
 * Math library spatial rotation related operations.
 */

#include <stdlib.h>
#include <math.h>

#include "spatial_rotations.h"

void
rotate_xyz_about_x( xyz_t * b, const xyz_t * const a, const double rot_angle)
{
  double cos_theta = cos(rot_angle);
  double sin_theta = sin(rot_angle);

  b->x =  a->x;
  b->y =  a->y*cos_theta + a->z*sin_theta;
  b->z = -a->y*sin_theta + a->z*cos_theta;
}

void
euler321_of_quat(euler_t *e, const quat_t * const q)
{
  double r11 = q->q0*q->q0 + q->q1*q->q1 - q->q2*q->q2 - q->q3*q->q3;
  double r12 = 2.0*(q->q1*q->q2 + q->q0*q->q3);
  double mr13 = -2.0*(q->q1*q->q3 - q->q0*q->q2);
  double r23 = 2.0*(q->q2*q->q3 + q->q0*q->q1);
  double r33 = q->q0*q->q0 - q->q1*q->q1 - q->q2*q->q2 + q->q3*q->q3;

  e->yaw   = atan2( r12, r11 );
  if (mr13 >  1.0) mr13 =  1.0; // nan protect
  else if (mr13 < -1.0) mr13 = -1.0;
  e->pitch = asin( mr13 );
  e->roll  = atan2( r23, r33 );
}

void
quat_of_dcm_a2b(quat_t * q, const double * const R)
{
  euler_t e;
  euler321_of_dcm( &e, R);
  quat_of_euler321( q, &e);
}

void
quat_of_dcm_b2a(quat_t * q, const double * const R)
{
  quat_of_dcm_a2b(q, R);
  quat_inv(q, q);
}


void
euler321_of_dcm(euler_t *e, const double * const R)
{
  double r11 = R[0];
  double r12 = R[1];
  double mr13 = -R[2];
  double r23 = R[5];
  double r33 = R[8];

  e->yaw   = atan2( r12, r11 );
  if (mr13 >  1.0) mr13 =  1.0; // nan protect
  else if (mr13 < -1.0) mr13 = -1.0;
  e->pitch = asin( mr13 );
  e->roll  = atan2( r23, r33 );
}

void
quat_of_euler321(quat_t * q, const euler_t * const e)
{
  double sr2 = sin(0.5*e->roll);
  double cr2 = cos(0.5*e->roll);
  double sp2 = sin(0.5*e->pitch);
  double cp2 = cos(0.5*e->pitch);
  double sy2 = sin(0.5*e->yaw);
  double cy2 = cos(0.5*e->yaw);
  q->q0 = cr2*cp2*cy2 + sr2*sp2*sy2;
  q->q1 = sr2*cp2*cy2 - cr2*sp2*sy2;
  q->q2 = cr2*sp2*cy2 + sr2*cp2*sy2;
  q->q3 = cr2*cp2*sy2 - sr2*sp2*cy2;

  if (q->q0 < 0){
    q->q0 = -q->q0;
    q->q1 = -q->q1;
    q->q2 = -q->q2;
    q->q3 = -q->q3;
  }

  quat_normalize(q);
}

void
dcm_of_quat_a2b(double *R_a2b, const quat_t * const q_a2b)
{
  // 1st column
  R_a2b[0] = q_a2b->q0*q_a2b->q0 + q_a2b->q1*q_a2b->q1 - q_a2b->q2*q_a2b->q2 - q_a2b->q3*q_a2b->q3;
  R_a2b[3] = 2*(q_a2b->q1*q_a2b->q2 - q_a2b->q0*q_a2b->q3);
  R_a2b[6] = 2*(q_a2b->q1*q_a2b->q3 + q_a2b->q0*q_a2b->q2);

  // 2nd column
  R_a2b[1] = 2*(q_a2b->q1*q_a2b->q2 + q_a2b->q0*q_a2b->q3);
  R_a2b[4] = q_a2b->q0*q_a2b->q0 - q_a2b->q1*q_a2b->q1 + q_a2b->q2*q_a2b->q2 - q_a2b->q3*q_a2b->q3;
  R_a2b[7] = 2*(q_a2b->q2*q_a2b->q3 - q_a2b->q0*q_a2b->q1);

  // 3rd column
  R_a2b[2] = 2*(q_a2b->q1*q_a2b->q3 - q_a2b->q0*q_a2b->q2);
  R_a2b[5] = 2*(q_a2b->q2*q_a2b->q3 + q_a2b->q0*q_a2b->q1);
  R_a2b[8] = q_a2b->q0*q_a2b->q0 - q_a2b->q1*q_a2b->q1 - q_a2b->q2*q_a2b->q2 + q_a2b->q3*q_a2b->q3;
}

void
dcm_of_quat_b2a(double *R_b2a, const quat_t * const q_a2b)
{
  quat_t q_b2a;
  q_b2a.q0 =  q_a2b->q0;
  q_b2a.q1 = -q_a2b->q1;
  q_b2a.q2 = -q_a2b->q2;
  q_b2a.q3 = -q_a2b->q3;

  dcm_of_quat_a2b(R_b2a, &q_b2a);
}

// vec_b = R_a2b * vec_a
void
rot_vec_by_dcm_a2b(xyz_t *vec_b, const double * const R_a2b, const xyz_t * const vec_a)
{
  xyz_mult_3x3_by_xyz( vec_b, R_a2b, vec_a );
}

// vec_a = R_a2b^T * vec_b
void
rot_vec_by_dcm_b2a(xyz_t *vec_a, const double * const R_a2b, const xyz_t * const vec_b)
{
  xyz_mult_3x3_transpose_by_xyz( vec_a, R_a2b, vec_b );
}

/* vec_b = q_a2b * vec_a * q_a2b^(-1) */
/* vec_b = R(q_a2b) * vec_a */
void
rot_vec_by_quat_a2b(xyz_t *vec_b, const quat_t * const q_a2b, const xyz_t * const vec_a)
{
  double R_a2b[9];
  dcm_of_quat_a2b(R_a2b,q_a2b);
  rot_vec_by_dcm_a2b(vec_b, R_a2b, vec_a);
}

void
rot_vec_by_quat_b2a(xyz_t *vec_a, const quat_t * const q_a2b, const xyz_t * const vec_b)
{
  double R_a2b[9];
  dcm_of_quat_a2b(R_a2b,q_a2b);
  rot_vec_by_dcm_b2a(vec_a, R_a2b, vec_b);
}

void
get_wind_angles_from_v_bw_b(double * alpha, double * beta, double * airspeed, const xyz_t * const v_bw_b)
{
  double airspeed_internal_memory;
  double * airspeed_internal = &airspeed_internal_memory;

  if (airspeed != NULL)
    *airspeed = xyz_norm(v_bw_b) + 1e-12;

  if (beta != NULL)
  {
    if (airspeed != NULL)
      airspeed_internal = airspeed;
    else
      *airspeed_internal = xyz_norm(v_bw_b) + 1e-12;

    *beta  =  asin ( v_bw_b->y / *airspeed_internal );
  }

  if (alpha != NULL)
    *alpha =  atan2( v_bw_b->z, v_bw_b->x );
}

void
get_wind_angles( double * alpha,
                 double * beta,
                 double * airspeed,
                 xyz_t * v_bw_b_out,
                 const quat_t * const q_n2b,
                 const xyz_t * const v_bn_b,
                 const xyz_t * const v_wn_n)
{
  xyz_t v_wn_b;
  rot_vec_by_quat_a2b( &v_wn_b, q_n2b, v_wn_n);
  xyz_t v_bw_b;
  xyz_diff( &v_bw_b, v_bn_b, &v_wn_b);

  get_wind_angles_from_v_bw_b( alpha, beta, airspeed, &v_bw_b );

  if (v_bw_b_out != NULL)
    xyz_memcpy( v_bw_b_out, &v_bw_b);
}

void
v_bw_b_from_wind_angles( xyz_t * v_bw_b, const double alpha, const double beta, const double airspeed)
{
  v_bw_b->x = airspeed*cos(alpha)*cos(beta);
  v_bw_b->y = airspeed*sin(beta);
  v_bw_b->z = airspeed*cos(beta)*sin(alpha);
}
