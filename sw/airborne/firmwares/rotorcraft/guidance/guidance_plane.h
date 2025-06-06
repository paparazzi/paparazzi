/*
 * Copyright (C) 2025 Gautier Hattenberger <gautier.hattenberger@enac.fr>
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
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */

/** @file firmwares/rotorcraft/guidance/guidance_plane.h
 *  Guidance controller for planes in rotorcraft firmware
 *  using basic PID controller
 *  no airspeed control
 *
 */

#ifndef GUIDANCE_PLANE_H
#define GUIDANCE_PLANE_H

#ifdef __cplusplus
extern "C" {
#endif

#include "std.h"
#include "math/pprz_algebra_int.h"
#include "math/pprz_algebra_float.h"

struct GuidancePlane {
  // horizontal
  float roll_max_setpoint;
  float pitch_max_setpoint;
  float pitch_min_setpoint;
  float course_setpoint;
  // horizontal gains
  float course_kp;
  float course_kd;
  float course_pre_bank_correction;

  // vertical
  float altitude_setpoint;
  float climb_max_setpoint;
  float climb_setpoint;
  float pitch_trim;
  // vertical gains, pitch and throttle
  float cruise_throttle;
  float climb_kp;
  float pitch_of_vz;
  float climb_throttle_increment;
  float p_kp;
  float p_kd;
  float p_ki;
  float t_kp;
  float t_kd;
  float t_ki;

  // outputs
  float roll_cmd;
  float pitch_cmd;
  int32_t throttle_cmd;           // Throttle command (in pprz_t)
};

/** Guidance PID structyre
 */
extern struct GuidancePlane guidance_plane;

extern void guidance_plane_init(void);
extern void guidance_plane_enter(void);
extern struct StabilizationSetpoint guidance_plane_attitude_from_nav(bool in_flight);
extern struct ThrustSetpoint guidance_plane_thrust_from_nav(bool in_flight);

#ifdef __cplusplus
}
#endif

#endif /* GUIDANCE_PLANE_H */
