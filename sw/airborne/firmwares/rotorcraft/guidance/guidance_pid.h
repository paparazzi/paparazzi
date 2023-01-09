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

struct GuidancePIDGains {
  // horizontal gains
  int32_t p;
  int32_t d;
  int32_t i;
  int32_t v;
  int32_t a;
  // vertical gains
  int32_t v_p;
  int32_t v_d;
  int32_t v_i;
};

/** Control gains
 */
extern struct GuidancePIDGains guidance_pid_gains;

/** horizontal guidance command.
 * In north/east with #INT32_ANGLE_FRAC
 * @todo convert to real force command
 */
extern struct Int32Vect2  guidance_pid_cmd_earth;

/** Correction of force commands from thrust
 */
extern bool guidance_pid_approx_force_by_thrust;


extern void guidance_pid_init(void);
extern struct Int32Vect2 guidance_pid_run_pos(bool in_flight);
extern struct Int32Vect2 guidance_pid_run_speed(bool in_flight);
extern struct Int32Vect2 guidance_pid_run_accel(bool in_flight);

extern void guidance_pid_set_igain(uint32_t igain);

/** Gets the position error
 * @param none.
 * @return Pointer to a structure containing x and y position errors
 */
extern const struct Int32Vect2 *guidance_pid_get_pos_err(void);

#ifdef __cplusplus
}
#endif

#endif /* GUIDANCE_PID_H */
