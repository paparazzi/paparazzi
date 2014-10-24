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

/**
 * @file stabilization_attitude_euler_float.h
 *
 * Rotorcraft attitude stabilization in euler float version.
 */

#ifndef STABILIZATION_ATTITUDE_EULER_FLOAT_H
#define STABILIZATION_ATTITUDE_EULER_FLOAT_H

#include "math/pprz_algebra_float.h"
#include "firmwares/rotorcraft/stabilization/stabilization_attitude_common_float.h"
#include "firmwares/rotorcraft/stabilization/stabilization_attitude_ref_euler_float.h"

extern struct FloatAttitudeGains stabilization_gains;
extern struct FloatEulers stabilization_att_sum_err;

#endif /* STABILIZATION_ATTITUDE_EULER_FLOAT_H */
