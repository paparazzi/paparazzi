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
#ifndef STABILIZATION_ATTITUDE_REF_FLOAT_H
#define STABILIZATION_ATTITUDE_REF_FLOAT_H

#include "generated/airframe.h"

#include "state.h"

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

static inline void reset_psi_ref_from_body(void) {
//sp has been set from body using stabilization_attitude_get_yaw_f, use that value
  stab_att_ref_euler.psi = stab_att_sp_euler.psi;
  stab_att_ref_rate.r = 0;
  stab_att_ref_accel.r = 0;
}

#endif /* STABILIZATION_ATTITUDE_REF_FLOAT_H */
