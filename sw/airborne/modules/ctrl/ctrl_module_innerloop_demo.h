/*
 * Copyright (C) 2015
 *
 * This file is part of Paparazzi.
 *
 * Paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * Paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */

/**
 * @file modules/ctrl/ctrl_module_innerloop_demo.c
 * @brief example empty controller
 *
 * Implements an example simple rate controller in a module.
 */

#ifndef CTRL_MODULE_INNERLOOP_DEMO_H_
#define CTRL_MODULE_INNERLOOP_DEMO_H_

#include <std.h>

// Settings
extern float ctrl_module_demo_pr_ff_gain;  // Pitch/Roll
extern float ctrl_module_demo_pr_d_gain;
extern float ctrl_module_demo_y_ff_gain;   // Yaw
extern float ctrl_module_demo_y_d_gain;

extern void ctrl_module_init(void);

// Implement own loops
extern void guidance_module_enter(void);
extern void guidance_module_run(bool in_flight);

#endif /* CTRL_MODULE_INNERLOOP_DEMO_H_ */
