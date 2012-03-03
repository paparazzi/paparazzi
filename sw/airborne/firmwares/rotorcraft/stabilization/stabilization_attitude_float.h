/*
 * $Id: stabilization_attitude.h 3794 2009-07-24 22:01:51Z poine $
 *
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

#ifndef STABILIZATION_ATTITUDE_FLOAT_H
#define STABILIZATION_ATTITUDE_FLOAT_H

#include "math/pprz_algebra_float.h"

#include "generated/airframe.h"

struct FloatAttitudeGains {
  struct FloatVect3  p;
  struct FloatVect3  d;
  struct FloatVect3  dd;
  struct FloatVect3  rates_d;
  struct FloatVect3  i;
  struct FloatVect3  surface_p;
  struct FloatVect3  surface_d;
  struct FloatVect3  surface_dd;
  struct FloatVect3  surface_i;
};

extern struct FloatAttitudeGains stabilization_gains[];
extern struct FloatEulers stabilization_att_sum_err_eulers;

extern float stabilization_att_fb_cmd[COMMANDS_NB];
extern float stabilization_att_ff_cmd[COMMANDS_NB];

void stabilization_attitude_gain_schedule(uint8_t idx);

#endif /* STABILIZATION_ATTITUDE_FLOAT_H */
