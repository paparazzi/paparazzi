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
 * xyz.h
 * Math library vector operations.
 */

#ifndef __XYZ_H__
#define __XYZ_H__

#ifndef XYZ_T
#define XYZ_T
typedef struct xyz_t {
  double x;
  double y;
  double z;
} xyz_t;
#endif // XYZ_T

#define xyz_printf(xyz_in) printf(#xyz_in ": [% .6f, % .6f, % .6f]\n",\
                                  (xyz_in)->x, (xyz_in)->y, (xyz_in)->z)

#ifdef __cplusplus
extern "C"{
#endif

  void xyz_memcpy( xyz_t * x1, const xyz_t * const x0);
  void xyz_set_all( xyz_t * xyz, double val);
  void xyz_set( xyz_t * xyz, double x, double y, double z);
  void xyz_diff(xyz_t *diff, const xyz_t * const a, const xyz_t * const b);
  void xyz_sum(xyz_t *sum, const xyz_t * const a, const xyz_t * const b);
  void xyz_sum_scale(xyz_t *sum, const double sum_scale,
                     const xyz_t * const a, const double a_scale,
                     const xyz_t * const b, const double b_scale);
  void xyz_cross( xyz_t * c, const xyz_t * const a, const xyz_t * const b);
  double xyz_dot( const xyz_t * const a, const xyz_t * const b);
  double xyz_norm_squared( const xyz_t * const vec);
  double xyz_norm( const xyz_t * const vec);
  double xyz_distance( const xyz_t * const a, const xyz_t * const b );
  void xyz_scale( xyz_t * vec_out, double scale_factor, const xyz_t * const vec_in);
  void xyz_scale_self( xyz_t * vec, double scale_factor);
  void xyz_normalize( xyz_t * vec_out, const xyz_t * const vec_in);
  void xyz_normalize_self( xyz_t * vec );
  void xyz_normalize_to( xyz_t * vec_out, const double new_norm, const xyz_t * const vec_in);
  void xyz_normalize_self_to( xyz_t * vec, const double new_norm );
  void xyz_mult_3x3_by_xyz(xyz_t * v_out, const double * const M, const xyz_t * const v);
  void xyz_mult_3x3_transpose_by_xyz(xyz_t * v_out, const double * const M, const xyz_t * const v);

#ifdef __cplusplus
}
#endif


#endif // __XYZ_H__
