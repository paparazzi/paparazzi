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

/** @file stabilization_attitude_ref_float.h
 *  Rotorcraft attitude reference generation API.
 *  Common to all floating-point reference generators (euler and quaternion)
 */

#ifndef STABILIZATION_ATTITUDE_REF_FLOAT_H
#define STABILIZATION_ATTITUDE_REF_FLOAT_H

#include "math/pprz_algebra_float.h"

extern struct FloatEulers stab_att_sp_euler;
extern struct FloatQuat   stab_att_sp_quat;
extern struct FloatEulers stab_att_ref_euler;
extern struct FloatQuat   stab_att_ref_quat;
extern struct FloatRates  stab_att_ref_rate;
extern struct FloatRates  stab_att_ref_accel;

struct FloatRefModel {
  struct FloatRates omega;
  struct FloatRates zeta;
};

extern struct FloatRefModel stab_att_ref_model[];

extern void stabilization_attitude_ref_init(void);
extern void stabilization_attitude_ref_update(void);

#endif /* STABILIZATION_ATTITUDE_REF_FLOAT_H */
