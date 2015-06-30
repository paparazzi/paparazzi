/*
 * Copyright (C) 2006  Pascal Brisset, Antoine Drouin, Michel Gorraz
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
 * @file firmwares/fixedwing/stabilization/stabilization_attitude.h
 *
 * Fixed wing horizontal control.
 *
 */

#ifndef FW_H_CTL_H
#define FW_H_CTL_H

#include <inttypes.h>
#include "std.h"
#include "paparazzi.h"
#include "generated/airframe.h"

/* outer loop parameters */
extern float h_ctl_course_setpoint; /* rad, CW/north */
extern float h_ctl_course_pre_bank;
extern float h_ctl_course_pre_bank_correction;
extern float h_ctl_course_pgain;
extern float h_ctl_course_dgain;
extern float h_ctl_roll_max_setpoint;

#ifdef LOITER_TRIM
extern float v_ctl_auto_throttle_loiter_trim;
extern float v_ctl_auto_throttle_dash_trim;
#endif

/* roll and pitch disabling */
extern bool_t h_ctl_disabled;

/* AUTO1 rate mode */
extern bool_t h_ctl_auto1_rate;

/* inner roll loop parameters */
extern float  h_ctl_roll_setpoint;
extern float  h_ctl_roll_pgain;
extern pprz_t h_ctl_aileron_setpoint;
extern float  h_ctl_roll_slew;

#ifdef USE_AOA
/* Pitch mode */
#define H_CTL_PITCH_MODE_THETA  0
#define H_CTL_PITCH_MODE_AOA  1
extern uint8_t h_ctl_pitch_mode;
#endif

/* inner pitch loop parameters */
extern float  h_ctl_pitch_setpoint;
extern float  h_ctl_pitch_loop_setpoint;
extern float  h_ctl_pitch_pgain;
extern float  h_ctl_pitch_dgain;
extern pprz_t h_ctl_elevator_setpoint;

/* inner yaw loop parameters */
#if H_CTL_YAW_LOOP
extern float  h_ctl_yaw_rate_setpoint;
extern pprz_t h_ctl_rudder_setpoint;
#endif

/* inner CL loop parameters */
#if H_CTL_CL_LOOP
extern pprz_t h_ctl_flaps_setpoint;
#endif


/* inner loop pre-command */
extern float h_ctl_aileron_of_throttle;
extern float h_ctl_elevator_of_roll;

/* rate loop */

#ifdef H_CTL_RATE_LOOP
extern float h_ctl_roll_rate_mode;
extern float h_ctl_roll_rate_setpoint_pgain;
extern float h_ctl_roll_rate_pgain;
extern float h_ctl_hi_throttle_roll_rate_pgain;
extern float h_ctl_lo_throttle_roll_rate_pgain;
extern float h_ctl_roll_rate_igain;
extern float h_ctl_roll_rate_dgain;

#define stabilization_attitude_SetRollRatePGain(v) { h_ctl_hi_throttle_roll_rate_pgain = v; h_ctl_lo_throttle_roll_rate_pgain = v; }
#endif

extern void h_ctl_init(void);
extern void h_ctl_course_loop(void);
extern void h_ctl_attitude_loop(void);

extern float h_ctl_roll_attitude_gain;
extern float h_ctl_roll_rate_gain;

#endif /* FW_H_CTL_H */
