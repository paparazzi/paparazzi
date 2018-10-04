/*
 * Copyright (C) MAVLab
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
/**
 * @file "modules/ctrl/dronerace//dronerace.h"
 * @author MAVLab
 * Autonomous Drone Race
 */

#ifndef DRONERACE_H
#define DRONERACE_H

// run
extern void dronerace_init(void);

extern void dronerace_enter(void);  // 1 time
extern void dronerace_periodic(void);

// export
extern void dronerace_set_rc(float t, float x, float y, float z);
extern void dronerace_get_cmd(float* alt, float* phi, float* theta, float* psi);

#endif

