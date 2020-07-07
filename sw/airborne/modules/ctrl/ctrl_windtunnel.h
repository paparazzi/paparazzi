/*
 * Copyright (C) 2020 Freek van Tienen
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
 * @file modules/ctrl/ctrl_windtunnel.h
 * @brief Windtunnel controller
 *
 * Implements a controller for the windtunnel which does some step controls
 */

#ifndef CTRL_MODULE_WINDTUNNEL_H_
#define CTRL_MODULE_WINDTUNNEL_H_

#include <std.h>

struct min_max_ctrl_t {
  float min;
  float max;
  float step;
  float current;
};

// Settings
extern float ctrl_windtunnel_steptime;
extern struct min_max_ctrl_t ctrl_windtunnel_throttle;
extern struct min_max_ctrl_t ctrl_windtunnel_flaps;

// Implement own Horizontal loops
extern void guidance_h_module_init(void);
extern void guidance_h_module_enter(void);
extern void guidance_h_module_read_rc(void);
extern void guidance_h_module_run(bool in_flight);

// Implement own Vertical loops
extern void guidance_v_module_init(void);
extern void guidance_v_module_enter(void);
extern void guidance_v_module_run(bool in_flight);

#endif /* CTRL_MODULE_WINDTUNNEL_H_ */
