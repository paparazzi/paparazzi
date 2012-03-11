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
 * quat.c
 * Math library for quaternion operations
 */

#include <math.h>
#include <string.h>

#include "quat.h"

void
quat_memcpy( quat_t * q1, const quat_t * const q0)
{
  memcpy( q1, q0, sizeof(quat_t) );
}

// q_out = q_in^-1
void
quat_inv(quat_t *q_out, const quat_t * const q_in)
{
  q_out->q0 =  q_in->q0;
  q_out->q1 = -q_in->q1;
  q_out->q2 = -q_in->q2;
  q_out->q3 = -q_in->q3;
}

// return ||q||
double
quat_norm(const quat_t * const q)
{
  return sqrt( q->q0*q->q0 + q->q1*q->q1 + q->q2*q->q2 + q->q3*q->q3);
}

// q /= ||q||
void
quat_normalize(quat_t * q)
{
  double norm_inv = 1.0/quat_norm(q);

  q->q0 = q->q0*norm_inv;
  q->q1 = q->q1*norm_inv;
  q->q2 = q->q2*norm_inv;
  q->q3 = q->q3*norm_inv;
}

// q_out = qa * qb
void
quat_mult(quat_t *q_out, const quat_t * const qa, const quat_t * const qb)
{
  q_out->q0 = qa->q0*qb->q0 - qa->q1*qb->q1 - qa->q2*qb->q2 - qa->q3*qb->q3;
  q_out->q1 = qa->q0*qb->q1 + qb->q0*qa->q1 + qa->q2*qb->q3 - qa->q3*qb->q2;
  q_out->q2 = qa->q0*qb->q2 + qb->q0*qa->q2 - qa->q1*qb->q3 + qa->q3*qb->q1;
  q_out->q3 = qa->q0*qb->q3 + qb->q0*qa->q3 + qa->q1*qb->q2 - qa->q2*qb->q1;

  if (q_out->q0 < 0){
    q_out->q0 = -q_out->q0;
    q_out->q1 = -q_out->q1;
    q_out->q2 = -q_out->q2;
    q_out->q3 = -q_out->q3;
  }

  quat_normalize( q_out );
}

// q_out = qa * qb^-1
void
quat_mult_inv(quat_t *q_out, const quat_t * const qa, const quat_t * const qb)
{
  quat_t qb_inv;
  quat_inv(&qb_inv,qb);
  quat_mult(q_out, qa, &qb_inv);
}

// q_out = qa^-1 * qb
void
quat_inv_mult(quat_t *q_out, const quat_t * const qa, const quat_t * const qb)
{
  quat_t qa_inv;
  quat_inv(&qa_inv,qa);
  quat_mult(q_out, &qa_inv, qb);
}

quat_t
quat_mult_ret(const quat_t qa, const quat_t qb)
{
  quat_t quat_out;
  quat_mult(&quat_out, &qa, &qb);
  return quat_out;
}
