/*
 * Copyright (C) 2013 Felix Ruess <felix.ruess@gmail.com>
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

/** @file stabilization_attitude_quat_transformations.h
 *  Quaternion transformation functions.
 */

#ifndef STABILIZATION_ATTITUDE_QUAT_TRANSFORMATIONS_H
#define STABILIZATION_ATTITUDE_QUAT_TRANSFORMATIONS_H

#include "math/pprz_algebra_float.h"
#include "math/pprz_algebra_int.h"

extern void quat_from_rpy_cmd_i(struct Int32Quat *quat, struct Int32Eulers *rpy);
extern void quat_from_rpy_cmd_f(struct FloatQuat *quat, struct FloatEulers *rpy);

extern void quat_from_earth_cmd_i(struct Int32Quat *quat, struct Int32Vect2 *cmd, int32_t heading);
extern void quat_from_earth_cmd_f(struct FloatQuat *quat, struct FloatVect2 *cmd, float heading);

#endif
