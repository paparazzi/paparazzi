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
 * along with Paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 */

/*
 * @file paparazzi/sw/airborne/modules/ctrl/ctrl_module_demo.h
 * @brief example empty controller
 *
 */

#ifndef CTRL_MODULE_DEMO_H_
#define CTRL_MODULE_DEMO_H_

#include <std.h>

// Demo Controller Module
extern void my_ctrl_init(void);
extern void my_ctrl_run(void);
// Settings
extern int ctrl_module_demo_gain;





// Vertical loop re-uses Alt-hold
#define GUIDANCE_V_MODE_MODULE_SETTING GUIDANCE_V_MODE_MODULE

// Horizontal mode is a specific controller
#define GUIDANCE_H_MODE_MODULE_SETTING GUIDANCE_H_MODE_MODULE

// Implement own Horizontal loops
inline void guidance_h_module_enter(void) { my_ctrl_init();}
inline void guidance_h_module_read_rc(void) {}
inline void guidance_h_module_run(bool_t in_flight) {my_ctrl_run();}

// Implement own Horizontal loops
inline void guidance_v_module_enter(void){}
inline void guidance_v_module_run(bool_t in_flight){}


#endif /* HOVER_STABILIZATION_H_ */
