/*
 * Copyright (C) 2017 Tom van Dijk <tomvand@users.noreply.github.com>
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
 * @file nps_fdm_gazebo.cpp
 * Flight Dynamics Model (FDM) for NPS using Gazebo.
 *
 * This is an FDM for NPS that uses Gazebo as the simulation engine.
 */

#include <cstdio>

#include "nps_fdm.h"

static void initialize_fdm_struct(void);

/// Holds all necessary NPS FDM state information
struct NpsFdm fdm;

// External functions, interface with Paparazzi's NPS as declared in nps_fdm.h

void nps_fdm_init(double dt) {
	printf("nps_fdm_init\n\n");
	initialize_fdm_struct();
}

void nps_fdm_run_step(bool launch, double *commands, int commands_nb) {
	printf("nps_fdm_run_step\n");
}
void nps_fdm_set_wind(double speed, double dir) {
}
void nps_fdm_set_wind_ned(
		double wind_north,
		double wind_east,
		double wind_down) {
}
void nps_fdm_set_turbulence(double wind_speed, int turbulence_severity) {
}
/** Set temperature in degrees Celcius at given height h above MSL */
void nps_fdm_set_temperature(double temp, double h) {
}

// Internal functions
static void initialize_fdm_struct(void) {
	// XXX Temporary, should be replaced by fetch() function to properly
	// initialize using Gazebo's state!
	fdm.time = 0;
	fdm.init_dt = 0;
	fdm.curr_dt = 0;
	fdm.on_ground = true;
	fdm.nan_count = 0;

	/* position */
	fdm.ecef_pos = {0, 0, 0};
	fdm.ltpprz_pos = {0, 0, 0};
	fdm.lla_pos = {0, 0, 0};
	// for debugginf
	fdm.lla_pos_pprz = {0, 0, 0};
	fdm.lla_pos_geod = {0, 0, 0};
	fdm.lla_pos_geoc = {0, 0, 0};
	fdm.agl = 0;

	/* velocity in ECEF wrt ECEF */
	fdm.ecef_ecef_vel = {0, 0, 0};
	/* accel in ECEF wrt ECEF */
	fdm.ecef_ecef_accel = {0, 0, 0};

	/* velocity in BODY wrt ECEF */
	fdm.body_ecef_vel = {0, 0, 0};
	/* accel in BODY wrt ECEF */
	fdm.body_ecef_accel = {0, 0, 0};

	/* velocity in LTP wrt ECEF */
	fdm.ltp_ecef_vel = {0, 0, 0};
	/* accel in LTP wrt ECEF */
	fdm.ltp_ecef_accel = {0, 0, 0};

	/* velocity in LTPPRZ wrt ECEF */
	fdm.ltpprz_ecef_vel = {0, 0, 0};
	/* accel in LTPPRZ wrt ECEF */
	fdm.ltpprz_ecef_accel = {0, 0, 0};

	/* accel in BODY wrt ECI */
	fdm.body_inertial_accel = {0, 0, 0};

	/* accel in BODY as measured by accelerometer (incl. gravity) */
	fdm.body_accel = {0, 0, 9.81};

	/* Attitude */
	// TODO
}

