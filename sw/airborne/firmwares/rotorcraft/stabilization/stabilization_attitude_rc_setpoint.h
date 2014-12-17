/*
 * Copyright (C) 2012 Felix Ruess <felix.ruess@gmail.com>
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

/** @file stabilization_attitude_rc_setpoint.h
 *  Read an attitude setpoint from the RC.
 */

#ifndef STABILIZATION_ATTITUDE_RC_SETPOINT_H
#define STABILIZATION_ATTITUDE_RC_SETPOINT_H

#include "std.h"
#include "math/pprz_algebra_int.h"
#include "math/pprz_algebra_float.h"

extern void stabilization_attitude_reset_care_free_heading(void);
extern int32_t stabilization_attitude_get_heading_i(void);
extern float stabilization_attitude_get_heading_f(void);
extern void stabilization_attitude_read_rc_setpoint_eulers(struct Int32Eulers *sp, bool_t in_flight, bool_t in_carefree,
    bool_t coordinated_turn);
extern void stabilization_attitude_read_rc_setpoint_eulers_f(struct FloatEulers *sp, bool_t in_flight,
    bool_t in_carefree, bool_t coordinated_turn);
extern void stabilization_attitude_read_rc_roll_pitch_quat_f(struct FloatQuat *q);
extern void stabilization_attitude_read_rc_roll_pitch_earth_quat_f(struct FloatQuat *q);
extern void stabilization_attitude_read_rc_setpoint_quat_f(struct FloatQuat *q_sp, bool_t in_flight, bool_t in_carefree,
    bool_t coordinated_turn);
extern void stabilization_attitude_read_rc_setpoint_quat_earth_bound_f(struct FloatQuat *q_sp, bool_t in_flight,
    bool_t in_carefree, bool_t coordinated_turn);

#endif /* STABILIZATION_ATTITUDE_RC_SETPOINT_H */
