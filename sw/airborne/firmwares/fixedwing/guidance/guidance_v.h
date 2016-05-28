/*
 * Copyright (C) 2006  Pascal Brisset, Antoine Drouin
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
 * @file firmwares/fixedwing/guidance/guidance_v.h
 * Vertical control for fixed wing vehicles.
 *
 */

#ifndef FW_V_CTL_H
#define FW_V_CTL_H

/* include common mode and variables definitions */
#include "firmwares/fixedwing/guidance/guidance_common.h"

/* outer loop */
//extern float v_ctl_altitude_error; in common definition
extern float v_ctl_altitude_setpoint; ///< in meters above MSL
extern float v_ctl_altitude_pre_climb;
extern float v_ctl_altitude_pgain;
extern float v_ctl_altitude_pre_climb_correction;
extern float v_ctl_altitude_max_climb;

/* "auto throttle" inner loop parameters */
extern float v_ctl_auto_throttle_nominal_cruise_throttle;
extern float v_ctl_auto_throttle_min_cruise_throttle;
extern float v_ctl_auto_throttle_max_cruise_throttle;
//extern float v_ctl_auto_throttle_cruise_throttle; in common definition
extern float v_ctl_auto_throttle_climb_throttle_increment;
extern float v_ctl_auto_throttle_pgain;
extern float v_ctl_auto_throttle_igain;
extern float v_ctl_auto_throttle_dgain;
//extern float v_ctl_auto_throttle_sum_err; in common definition
extern float v_ctl_auto_throttle_pitch_of_vz_pgain;
extern float v_ctl_auto_throttle_pitch_of_vz_dgain;

/* cruise pitch trim */
extern float v_ctl_pitch_trim;

/* agressive tuning */
#ifdef TUNE_AGRESSIVE_CLIMB
extern float agr_climb_throttle;
extern float agr_climb_pitch;
extern float agr_climb_nav_ratio;
extern float agr_descent_throttle;
extern float agr_descent_pitch;
extern float agr_descent_nav_ratio;
#endif

/* "auto pitch" inner loop parameters */
extern float v_ctl_auto_pitch_pgain;
extern float v_ctl_auto_pitch_igain;
extern float v_ctl_auto_pitch_sum_err;

#if USE_AIRSPEED
/* "airspeed" inner loop parameters */
extern float v_ctl_auto_airspeed_setpoint;
//extern float v_ctl_auto_airspeed_controlled; in common definition
extern float v_ctl_auto_airspeed_pgain;
extern float v_ctl_auto_airspeed_igain;
extern float v_ctl_auto_airspeed_sum_err;
//extern float v_ctl_auto_groundspeed_setpoint; in common definition
extern float v_ctl_auto_groundspeed_pgain;
extern float v_ctl_auto_groundspeed_igain;
extern float v_ctl_auto_groundspeed_sum_err;
#endif

#if CTRL_VERTICAL_LANDING
extern float v_ctl_landing_throttle_pgain;
extern float v_ctl_landing_throttle_igain;
extern float v_ctl_landing_throttle_max;
extern float v_ctl_landing_desired_speed;
extern float v_ctl_landing_pitch_pgain;
extern float v_ctl_landing_pitch_igain;
extern float v_ctl_landing_pitch_limits;
extern float v_ctl_landing_pitch_flare;
extern float v_ctl_landing_alt_throttle_kill;
extern float v_ctl_landing_alt_flare;
#endif

#endif /* FW_V_CTL_H */
