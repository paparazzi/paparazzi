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
 * xyz.c
 * Math library vector operations.
 */

#include <math.h>
#include <string.h>

#include "xyz.h"

void
xyz_memcpy( xyz_t * x1, const xyz_t * const x0)
{
  memcpy( x1, x0, sizeof(xyz_t) );
}

void
xyz_set_all( xyz_t * xyz, double val)
{
  xyz->x = val;
  xyz->y = val;
  xyz->z = val;
}

void
xyz_set( xyz_t * xyz, double x, double y, double z)
{
  xyz->x = x;
  xyz->y = y;
  xyz->z = z;
}

// diff = a - b
void
xyz_diff(xyz_t *diff, const xyz_t * const a, const xyz_t * const b)
{
  diff->x = a->x - b->x;
  diff->y = a->y - b->y;
  diff->z = a->z - b->z;
}

// sum = a + b
void
xyz_sum(xyz_t *sum, const xyz_t * const a, const xyz_t * const b)
{
  sum->x = a->x + b->x;
  sum->y = a->y + b->y;
  sum->z = a->z + b->z;
}

// sum = sum*sum_scale + a*a_scale + b*b_scale
void
xyz_sum_scale(xyz_t *sum, const double sum_scale,
              const xyz_t * const a, const double a_scale,
              const xyz_t * const b, const double b_scale)
{
  sum->x = sum->x*sum_scale + a->x*a_scale + b->x*b_scale;
  sum->y = sum->y*sum_scale + a->y*a_scale + b->y*b_scale;
  sum->z = sum->z*sum_scale + a->z*a_scale + b->z*b_scale;
}

// c = a (cross) b
void
xyz_cross( xyz_t * c, const xyz_t * const a, const xyz_t * const b)
{
  c->x =   a->y*b->z - a->z*b->y;
  c->y = - a->x*b->z + a->z*b->x;
  c->z =   a->x*b->y - a->y*b->x;
}

// return a (dot) b
double
xyz_dot( const xyz_t * const a, const xyz_t * const b)
{
  return a->x*b->x + a->y*b->y + a->z*b->z;
}

// return vec (dot) vec
double
xyz_norm_squared( const xyz_t * const vec)
{
  return xyz_dot( vec, vec);
}

// return norm(vec)
double
xyz_norm( const xyz_t * const vec)
{
  return sqrt(xyz_norm_squared(vec));
}

// return norm(a - b)
double
xyz_distance( const xyz_t * const a, const xyz_t * const b )
{
  xyz_t diff;
  xyz_diff( &diff, a, b);
  return xyz_norm( &diff );
}

// vec_out = vec_in*scale_factor
void
xyz_scale( xyz_t * vec_out, double scale_factor, const xyz_t * const vec_in)
{
  vec_out->x = scale_factor*vec_in->x;
  vec_out->y = scale_factor*vec_in->y;
  vec_out->z = scale_factor*vec_in->z;
}

// vec *= scale_factor
void
xyz_scale_self( xyz_t * vec, double scale_factor)
{
  xyz_scale(vec, scale_factor, vec);
}

// vec_out = vec_in/norm(vec_in)
void
xyz_normalize( xyz_t * vec_out, const xyz_t * const vec_in)
{
  xyz_normalize_to( vec_out, 1.0, vec_in);
}

// vec *= 1.0/norm(vec)
void
xyz_normalize_self( xyz_t * vec )
{
  xyz_normalize_self_to( vec, 1.0 );
}

// vec_out = vec_in*new_norm/norm(vec_in)
void
xyz_normalize_to( xyz_t * vec_out, const double new_norm, const xyz_t * const vec_in)
{
  xyz_scale( vec_out, new_norm/(xyz_norm(vec_in) + 1e-12), vec_in );
}

// vec *= new_norm/norm(vec)
void
xyz_normalize_self_to( xyz_t * vec, const double new_norm )
{
  xyz_scale_self( vec, new_norm/(xyz_norm(vec) + 1e-12) );
}

// v_out = M*v
// M is 3x3 row-major matrix
void
xyz_mult_3x3_by_xyz(xyz_t * v_out, const double * const M, const xyz_t * const v)
{
  v_out->x = M[0]*v->x +  M[1]*v->y +  M[2]*v->z;
  v_out->y = M[3]*v->x +  M[4]*v->y +  M[5]*v->z;
  v_out->z = M[6]*v->x +  M[7]*v->y +  M[8]*v->z;
}

// v_out = M^T*v
// M is 3x3 row-major matrix
void
xyz_mult_3x3_transpose_by_xyz(xyz_t * v_out, const double * const M, const xyz_t * const v)
{
  v_out->x = M[0]*v->x +  M[3]*v->y +  M[6]*v->z;
  v_out->y = M[1]*v->x +  M[4]*v->y +  M[7]*v->z;
  v_out->z = M[2]*v->x +  M[5]*v->y +  M[8]*v->z;
}
