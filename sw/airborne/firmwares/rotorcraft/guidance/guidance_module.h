/*
 * Copyright (C) 2015
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
 * along with paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 */

/**
 * @file firmwares/rotorcraft/guidance/guidance_module.h
 * Guidance in a module file.
 *
 * Implement a custom controller in a module.
 * Re-use desired modes:
 *
 * e.g.: <tt>#define GUIDANCE_V_MODE_MODULE_SETTING GUIDANCE_V_MODE_HOVER</tt>
 * can be used to only define a horizontal control in the module and use normal z_hold
 *
 * The guidance that the module implement must be activated with following defines:
 *
 * a) Implement own loops
 * - void guidance_module_enter(void);
 * - void guidance_module_run(bool in_flight);
 *
 *
 * b) Use or copy the generated autopilot file 'conf/autopilot/rotorcraft_control_loop.xml'
 * - load custom autopilot from your airframe filewith by adding
 *   <autopilot name="rotorcraft_control_loop"/>
 *   to the firmware section
 *
 * If the module implements both V and H mode, take into account that the V is called first and later H
 *
 */

#ifndef GUIDANCE_MODULE_H_
#define GUIDANCE_MODULE_H_

#include "generated/modules.h"

#endif /* GUIDANCE_MODULE_H_ */
