/*
 * Copyright (C) Joost Meulenbeld
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
 * @file "modules/helicopter/sys_id_chirp.h"
 * @author Joost Meulenbeld
 * System identification chirp
 * 
 * This is the module implementation for the chirp maneuver. The mathematical definition of the chirp
 * can be found in pprz_chirp.h. Use sys_id_chirp by adding the module to your airframe file and
 * adding the following line to the top of the <command_laws> section of your airframe file:
 * 
 * <call fun="sys_id_chirp_add_values(autopilot_get_motors_on(),FALSE,values)"/>
 * 
 * In the GCS you can then start and stop the chirp, change frequencies and choose which axis it should
 * be applied to. Documentation of the specific options can be found in the module xml file.
 * 
 * The axes to which noise is applied is set in the xml file with the variable CHIRP_AXES. The axis that
 * is selected in the GCS to apply the chirp on is the index in the CHIRP_AXES array
 */

#ifndef SYS_ID_CHIRP_H
#define SYS_ID_CHIRP_H

#include <std.h>
#include <stdbool.h>
#include "paparazzi.h"
#include "modules/system_identification/pprz_chirp.h"
#include "generated/airframe.h"
#include "mcu_periph/sys_time.h"
#include "filters/low_pass_filter.h"
#include "random.h"


#ifndef CHIRP_AXES
#define CHIRP_AXES {COMMAND_ROLL,COMMAND_PITCH,COMMAND_YAW}
#endif

#ifndef CHIRP_ENABLED
#define CHIRP_ENABLED TRUE
#endif

#ifndef CHIRP_USE_NOISE
#define CHIRP_USE_NOISE TRUE
#endif

#ifndef CHIRP_EXPONENTIAL
#define CHIRP_EXPONENTIAL TRUE
#endif

#ifndef CHIRP_FADEIN
#define CHIRP_FADEIN TRUE
#endif

extern uint8_t chirp_active;
extern pprz_t chirp_amplitude;
extern float chirp_noise_stdv_onaxis_ratio; // On-axis noise is amplitude times this value
extern float chirp_noise_stdv_offaxis; // Off-axis noise (the axes that the chirp is not applied to)

extern float chirp_fstart_hz;
extern float chirp_fstop_hz;
extern float chirp_length_s;

extern uint8_t chirp_axis; // Index of chirp axis in ACTIVE_CHIRP_AXES

extern void sys_id_chirp_init(void);

// If chirp is running, update its values
extern void sys_id_chirp_run(void);

// Handlers for changing gcs variables
extern void sys_id_chirp_activate_handler(uint8_t activate); // Activate the chirp
extern void sys_id_chirp_axis_handler(uint8_t axis); // Check if new axis is valid
extern void sys_id_chirp_fstart_handler(float fstart); // Check if fstart is lower than current fend
extern void sys_id_chirp_fstop_handler(float fstop); // Check if fend is higher than current fstart

// Add the current chirp values to the in_cmd values if motors_on is true
extern void sys_id_chirp_add_values(bool motors_on, bool override_on, pprz_t in_cmd[]);

#endif // SYS_ID_CHIRP_H
