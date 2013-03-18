/*
 * Copyright (C) 2012-2013 Freek van Tienen
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

#include "firmwares/rotorcraft/stabilization.h"
#include "firmwares/rotorcraft/stabilization/stabilization_attitude_rc_setpoint.h"
#include "paparazzi.h"

#include "subsystems/radio_control.h"
#include "generated/airframe.h"

#define TRAJ_MAX_BANK (int32_t)ANGLE_BFP_OF_REAL(GUIDANCE_H_MAX_BANK)

struct Int32Eulers stab_att_sp_euler;

//STUB
struct Int32Eulers stabilization_att_sum_err;
struct Int32Eulers stab_att_ref_euler; ///< with #REF_ANGLE_FRAC
struct Int32Rates  stab_att_ref_rate;  ///< with #REF_RATE_FRAC
struct Int32Rates  stab_att_ref_accel; ///< with #REF_ACCEL_FRAC
int32_t stabilization_att_fb_cmd[COMMANDS_NB];
int32_t stabilization_att_ff_cmd[COMMANDS_NB];

void stabilization_attitude_init(void) {
	INT_EULERS_ZERO(stabilization_att_sum_err);
	INT_EULERS_ZERO(stab_att_sp_euler);
	INT_EULERS_ZERO(stab_att_ref_euler);
	INT_RATES_ZERO(stab_att_ref_rate);
	INT_RATES_ZERO(stab_att_ref_accel);
}


void stabilization_attitude_read_rc(bool_t in_flight) {
	//Read from RC
	stabilization_attitude_read_rc_setpoint_eulers(&stab_att_sp_euler, in_flight);
}


void stabilization_attitude_enter(void) {

}

void stabilization_attitude_run(bool_t  in_flight __attribute__ ((unused))) {
	/* For roll an pitch we pass truough the desired angles as stabilization command */
	EULERS_SMUL(stab_att_ref_euler, stab_att_sp_euler, MAX_PPRZ/TRAJ_MAX_BANK);
	stabilization_cmd[COMMAND_ROLL] = stab_att_ref_euler.phi;
	stabilization_cmd[COMMAND_PITCH] = stab_att_ref_euler.theta;

	//TODO: Fix yaw with PID controller
	int32_t yaw_error = stateGetNedToBodyEulers_i()->psi-stab_att_sp_euler.psi;
	INT32_ANGLE_NORMALIZE(yaw_error);
//	stabilization_cmd[COMMAND_YAW] = yaw_error * MAX_PPRZ / INT32_ANGLE_PI;

	/* bound the result */
	BoundAbs(stabilization_cmd[COMMAND_ROLL], MAX_PPRZ);
	BoundAbs(stabilization_cmd[COMMAND_PITCH], MAX_PPRZ);
	BoundAbs(stabilization_cmd[COMMAND_YAW], MAX_PPRZ);
}

void stabilization_attitude_ref_init(void) {

}

void stabilization_attitude_ref_update(void) {

}
