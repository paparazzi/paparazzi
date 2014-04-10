/*
 * Copyright (C) 2008-2010 The Paparazzi Team
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

/** @file stabilization_attitude_ref_int.h
 *  Rotorcraft attitude reference generation API.
 *  Common to all fixed-point reference generators (euler and quaternion)
 */

#ifndef STABILIZATION_ATTITUDE_REF_INT_H
#define STABILIZATION_ATTITUDE_REF_INT_H

#include "math/pprz_algebra_int.h"

#define REF_ACCEL_FRAC 12
#define REF_RATE_FRAC  16
#define REF_ANGLE_FRAC 20

extern struct Int32Eulers stab_att_sp_euler; ///< with #INT32_ANGLE_FRAC
extern struct Int32Eulers stab_att_ref_euler; ///< with #REF_ANGLE_FRAC
extern struct Int32Rates  stab_att_ref_rate;  ///< with #REF_RATE_FRAC
extern struct Int32Rates  stab_att_ref_accel; ///< with #REF_ACCEL_FRAC

extern void stabilization_attitude_ref_init(void);
extern void stabilization_attitude_ref_update(void);

#endif /* STABILIZATION_ATTITUDE_REF_INT_H */
