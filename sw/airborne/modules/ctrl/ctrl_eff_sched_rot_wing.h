/*
 * Copyright (C) 2023 Dennis van Wijngaarden <D.C.vanWijngaarden@tudelft.nl>
 *
 * This file is part of paparazzi
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
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */

/** @file "modules/ctrl/ctrl_eff_sched_rot_wing.h"
 * @author Dennis van Wijngaarden <D.C.vanWijngaarden@tudelft.nl>
 * The control effectiveness scheduler for the rotating wing drone type
 */

#ifndef CTRL_EFF_SCHED_ROT_WING_H
#define CTRL_EFF_SCHED_ROT_WING_H

#include "std.h"

struct rot_wing_eff_sched_param_t {
  float Ixx_body;                 // body MMOI around roll axis [kgm²] 
  float Iyy_body;                 // body MMOI around pitch axis [kgm²] 
  float Izz;                      // total MMOI around yaw axis [kgm²]
  float Ixx_wing;                 // wing MMOI around the chordwise direction of the wing [kgm²]
  float Iyy_wing;                 // wing MMOI around the spanwise direction of the wing [kgm²]
  float m;                        // mass [kg]
  float hover_roll_pitch_coef[2]; // Model coefficients to correct pitch effective for roll motors
  float hover_roll_roll_coef[2];  // Model coefficients to correct roll effectiveness for roll motors
  float k_elevator[3];
  float k_rudder[3];
  float k_aileron;
  float k_flaperon;
  float k_pusher[2];
  float k_elevator_deflection[2];
  float d_rudder_d_pprz;
  float k_rpm_pprz_pusher[3];
  float k_lift_wing[2];
  float k_lift_fuselage;
  float k_lift_tail;
};

struct rot_wing_eff_sched_var_t {
  float Ixx;                  // Total MMOI around roll axis [kgm²]
  float Iyy;                  // Total MMOI around pitch axis [kgm²]
  float wing_rotation_rad;    // Wing rotation angle in radians
  float wing_rotation_deg;    // Wing rotation angle in degrees
  float cosr;                 // cosine of wing rotation angle
  float sinr;                 // sine of wing rotation angle
  float cosr2;                // cosine² of wing rotation angle
  float sinr2;                // sine² of wing rotation angle
  float sinr3;                // sine³ of wing rotation angle

  // Set during initialization
  float pitch_motor_dMdpprz;  // derivative of delta moment with respect to a delta paparazzi command for the pitch motors [Nm/pprz]
  float roll_motor_dMdpprz;   // derivative of delta moment with respect to a delta paparazzi command for the roll motors [Nm/pprz]

  // commands
  float cmd_elevator;
  float cmd_pusher_scaled;
  float cmd_T_mean_scaled;

  // airspeed
  float airspeed;
  float airspeed2;

};

extern float rotation_angle_setpoint_deg;
extern int16_t rotation_cmd;

extern void ctrl_eff_sched_rot_wing_init(void);
extern void ctrl_eff_sched_rot_wing_periodic(void);

#endif  // CTRL_EFF_SCHED_ROT_WING_H
