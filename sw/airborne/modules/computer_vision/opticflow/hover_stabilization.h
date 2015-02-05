/*
 * Copyright (C) 2014 Hann Woei Ho
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
 * @file modules/computer_vision/opticflow/hover_stabilization.h
 * @brief optical-flow based hovering for Parrot AR.Drone 2.0
 *
 * Control loops for optic flow based hovering.
 * Computes setpoint for the lower level attitude stabilization to control horizontal velocity.
 */

#ifndef HOVER_STABILIZATION_H_
#define HOVER_STABILIZATION_H_

#include <std.h>
#include "inter_thread_data.h"

// Controller module

// Vertical loop re-uses Alt-hold
#define GUIDANCE_V_MODE_MODULE_SETTING GUIDANCE_V_MODE_HOVER

// Horizontal mode is a specific controller
#define GUIDANCE_H_MODE_MODULE_SETTING GUIDANCE_H_MODE_MODULE

// Implement own Horizontal loops
extern void guidance_h_module_enter(void);
extern void guidance_h_module_read_rc(void);
extern void guidance_h_module_run(bool_t in_flight);


void init_hover_stabilization_onvision(void);
void run_hover_stabilization_onvision(struct CVresults *vision);

extern bool activate_opticflow_hover;
extern float vision_desired_vx;
extern float vision_desired_vy;
extern int32_t vision_phi_pgain;
extern int32_t vision_phi_igain;
extern int32_t vision_theta_pgain;
extern int32_t vision_theta_igain;

#endif /* HOVER_STABILIZATION_H_ */
