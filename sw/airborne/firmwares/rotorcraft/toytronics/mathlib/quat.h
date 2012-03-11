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
 * quat.h
 * Math library for quaternion operations
 */

#ifndef __QUAT_H__
#define __QUAT_H__

#ifndef QUAT_T
#define QUAT_T
typedef struct quat_t {
  double q0;
  double q1;
  double q2;
  double q3;
} quat_t;
#endif // QUAT_T

#define quat_printf(quat_in) printf(#quat_in ": [% .6f, % .6f, % .6f, % .6f]\n",\
                                    (quat_in)->q0, (quat_in)->q1, (quat_in)->q2, (quat_in)->q3)

#ifdef __cplusplus
extern "C"{
#endif

  void quat_memcpy( quat_t * q1, const quat_t * const q0);
  void quat_inv(quat_t *q_out, const quat_t * const q_in);
  double quat_norm(const quat_t * const q);
  void quat_normalize(quat_t * q);
  void quat_mult(quat_t *q_out, const quat_t * const qa, const quat_t * const qb);
  void quat_mult_inv(quat_t *q_out, const quat_t * const qa, const quat_t * const qb);
  void quat_inv_mult(quat_t *q_out, const quat_t * const qa, const quat_t * const qb);
  quat_t quat_mult_ret(const quat_t qa, const quat_t qb);

#ifdef __cplusplus
}
#endif

#endif // __QUAT_H__
