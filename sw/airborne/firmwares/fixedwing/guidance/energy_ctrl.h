/*
 * Copyright (C) 2012 TUDelft, Tobias Muench
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
 * @file firmwares/fixedwing/guidance/energy_ctrl.h
 * Vertical control using total energy control for fixed wing vehicles.
 *
 */

#ifndef FW_V_CTL_ENERGY_H
#define FW_V_CTL_ENERGY_H

#include "firmwares/fixedwing/guidance/guidance_common.h"

/* outer loop */
// extern float v_ctl_altitude_error;    ///< in meters, (setpoint - alt) -> positive = too low
extern float v_ctl_altitude_setpoint; ///< in meters above MSL
extern float v_ctl_altitude_pre_climb; ///< Path Angle
extern float v_ctl_altitude_pgain;
extern float v_ctl_airspeed_pgain;

extern float v_ctl_auto_airspeed_setpoint; ///< in meters per second

extern float v_ctl_max_climb;
extern float v_ctl_max_acceleration;

/* "auto throttle" inner loop parameters */
extern float v_ctl_desired_acceleration;

extern float v_ctl_auto_throttle_nominal_cruise_throttle;
extern float v_ctl_auto_throttle_nominal_cruise_pitch;
extern float v_ctl_auto_throttle_climb_throttle_increment;
extern float v_ctl_auto_throttle_pitch_of_vz_pgain;

extern float v_ctl_auto_throttle_of_airspeed_pgain;
extern float v_ctl_auto_throttle_of_airspeed_igain;
extern float v_ctl_auto_pitch_of_airspeed_pgain;
extern float v_ctl_auto_pitch_of_airspeed_igain;
extern float v_ctl_auto_pitch_of_airspeed_dgain;

extern float v_ctl_energy_total_pgain;
extern float v_ctl_energy_total_igain;

extern float v_ctl_energy_diff_pgain;
extern float v_ctl_energy_diff_igain;

extern float v_ctl_auto_groundspeed_pgain;
extern float v_ctl_auto_groundspeed_igain;
extern float v_ctl_auto_groundspeed_sum_err;

/////////////////////////////////////////////////
// Automatically found airplane characteristics

extern float ac_char_climb_pitch;
extern float ac_char_climb_max;
extern float ac_char_descend_pitch;
extern float ac_char_descend_max;
extern float ac_char_cruise_throttle;
extern float ac_char_cruise_pitch;

#endif /* FW_V_CTL_H */
