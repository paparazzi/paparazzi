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
  int32_t p;
  int32_t d;
  int32_t i;
  int32_t v;
  int32_t a;
};

extern void guidance_pid_init(void);
extern void guidance_pid_run_pos(bool in_flight);
extern void guidance_pid_run_speed(bool in_flight);
extern void guidance_pid_run_accel(bool in_flight);

#ifdef __cplusplus
}
#endif

#endif /* GUIDANCE_PID_H */
