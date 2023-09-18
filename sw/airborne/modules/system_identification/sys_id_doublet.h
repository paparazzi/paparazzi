/*
 * Copyright (C) Alessandro Collicelli
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
 * @file "modules/system_identification/sys_id_doublet.c"
 * @author Alessandro Collicelli
 * System identification doublet
 * 
 * This is the module implementation for the doublet maneuver. Use sys_id_chirp by adding the module to your airframe file and
 * adding the following line to the top of the <command_laws> section of your airframe file:
 *
 * <call fun="sys_id_doublet_add_values(autopilot_get_motors_on(),FALSE,values)"/>
 *
 * In the GCS you can then start and stop the doublet, change type of doublet maneuvre. 
 * Documentation of the specific options can be found in the module xml file.
 */


#ifndef SYS_ID_DOUBLET_H
#define SYS_ID_DOUBLET_H

#include "paparazzi.h"


extern uint8_t doublet_active;
extern pprz_t doublet_amplitude;

extern float doublet_length_s;
extern float doublet_extra_waiting_time_s;
// Index of doublet axis in ACTIVE_DOUBLET_AXES
extern uint8_t doublet_axis;

extern uint8_t doublet_mode;

extern void sys_id_doublet_init(void);

// If doublet is running, update its values
extern void sys_id_doublet_run(void);

// Handlers for changing gcs variables
extern void sys_id_doublet_activate_handler(uint8_t activate); // Activate the doublet
extern void sys_id_doublet_axis_handler(uint8_t axis); // Check if new axis
extern void sys_id_doublet_mod_handler(uint8_t mode);
extern uint8_t sys_id_doublet_running(void);
// Add the current doublet values to the in_cmd values if motors_on is true
extern void sys_id_doublet_add_values(bool motors_on, bool override_on, pprz_t in_cmd[]);

#endif // SYS_ID_DOUBLET_H
