/*
 * Copyright (C) 2008-2009 Antoine Drouin <poinix@gmail.com>
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

/** @file stabilization_attitude_ref_euler_int.h
 *  Rotorcraft attitude reference generation (euler int version)
 *
 */

#ifndef STABILIZATION_ATTITUDE_REF_EULER_INT_H
#define STABILIZATION_ATTITUDE_REF_EULER_INT_H

#include "stabilization_attitude_ref_int.h"

/** Attitude reference models and state/output (euler int) */
struct AttRefEulerInt {
  struct Int32Eulers euler;  ///< with #REF_ANGLE_FRAC
  struct Int32Rates  rate;   ///< with #REF_RATE_FRAC
  struct Int32Rates  accel;  ///< with #REF_ACCEL_FRAC
};

extern void attitude_ref_euler_int_init(struct AttRefEulerInt *ref);
extern void attitude_ref_euler_int_enter(struct AttRefEulerInt *ref, int32_t psi);
extern void attitude_ref_euler_int_update(struct AttRefEulerInt *ref, struct Int32Eulers *sp_euler);

#endif /* STABILIZATION_ATTITUDE_REF_EULER_INT_H */
