/*
 * Copyright (C) 2023 Gautier Hattenberger <gautier.hattenberger@enac.fr>
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

/** @file firmwares/rotorcraft/guidance/guidance_pid.h
 *  Guidance controller with PID for rotorcrafts.
 *
 */

#ifndef GUIDANCE_PID_H
#define GUIDANCE_PID_H

#ifdef __cplusplus
extern "C" {
#endif

#include "std.h"
#include "math/pprz_algebra_int.h"
#include "firmwares/rotorcraft/guidance/guidance_h.h"
#include "firmwares/rotorcraft/guidance/guidance_v.h"

struct GuidancePID {
  // horizontal gains
  int32_t kp;
  int32_t kd;
  int32_t ki;
  int32_t kv;
  int32_t ka;
  // vertical gains
  int32_t v_kp;
  int32_t v_kd;
  int32_t v_ki;
  // outputs
  struct Int32Vect2 cmd_earth;  // Horizontal guidance command (north/east with #INT32_ANGLE_FRAC)
  int32_t cmd_thrust;           // Vertical guidance command (in pprz_t)
  // options
  bool approx_force_by_thrust;  // Correction of force commands from thrust
  bool adapt_throttle_enabled;  // Use adaptive throttle command estimation
};

/** Guidance PID structyre
 */
extern struct GuidancePID guidance_pid;

extern void guidance_pid_init(void);
extern void guidance_pid_h_enter(void);
extern void guidance_pid_v_enter(void);
extern struct StabilizationSetpoint guidance_pid_h_run_pos(bool in_flight, struct HorizontalGuidance *gh);
extern struct StabilizationSetpoint guidance_pid_h_run_speed(bool in_flight, struct HorizontalGuidance *gh);
extern struct StabilizationSetpoint guidance_pid_h_run_accel(bool in_flight, struct HorizontalGuidance *gh);
extern int32_t guidance_pid_v_run_pos(bool in_flight, struct VerticalGuidance *gv);
extern int32_t guidance_pid_v_run_speed(bool in_flight, struct VerticalGuidance *gv);
extern int32_t guidance_pid_v_run_accel(bool in_flight, struct VerticalGuidance *gv);

extern void guidance_pid_set_h_igain(uint32_t igain);
extern void guidance_pid_set_v_igain(uint32_t igain);

/** Gets the position error
 * @param none.
 * @return Pointer to a structure containing x and y position errors
 */
extern const struct Int32Vect2 *guidance_pid_get_h_pos_err(void);

#ifdef __cplusplus
}
#endif

#endif /* GUIDANCE_PID_H */
