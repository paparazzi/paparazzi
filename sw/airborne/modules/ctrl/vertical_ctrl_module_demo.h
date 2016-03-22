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
 * @file modules/ctrl/vertical_ctrl_module_demo.h
 * @brief example vertical
 *
 * Implements an example vertical controller in a module.
 */

#ifndef VERTICAL_CTRL_MODULE_DEMO_H_
#define VERTICAL_CTRL_MODULE_DEMO_H_

#include "std.h"

struct VerticalCtrlDemo {
  float agl;
  float setpoint;
  float pgain;
  float igain;
  float sum_err;
};

extern struct VerticalCtrlDemo v_ctrl;

// for example use the standard horizontal (hover) mode
#define GUIDANCE_H_MODE_MODULE_SETTING GUIDANCE_H_MODE_HOVER

// and own guidance_v
#define GUIDANCE_V_MODE_MODULE_SETTING GUIDANCE_V_MODE_MODULE

// Implement own Vertical loops
extern void guidance_v_module_init(void);
extern void guidance_v_module_enter(void);
extern void guidance_v_module_run(bool in_flight);

#endif /* VERTICAL_CTRL_MODULE_DEMO_H_ */
