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
 * @file "modules/system_identification/sys_id_wave.c"
 * @author Alessandro Collicelli
 * System excitation pure sine wave
 * 
 * This is the module implementation for the a sine wave input. Use sys_id_wave by adding the module to your airframe file and
 * adding the following line to the top of the <command_laws> section of your airframe file:
 *
 * <call fun="sys_id_wave_add_values(autopilot_get_motors_on(),FALSE,values)"/>
 *
 * In the GCS you can then start and stop the wave, change wave frequency. 
 * Documentation of the specific options can be found in the module xml file.
 */

#ifndef SYS_ID_WAVE_H
#define SYS_ID_WAVE_H


#include "paparazzi.h"


extern uint8_t wave_active;
extern uint8_t wave_axis;
extern pprz_t wave_amplitude;
extern float frequency_hz_;
extern float lag_rad_;

extern uint8_t wave_axis;

extern void sys_id_wave_init(void);
extern void sys_id_wave_run(void);

extern void sys_id_wave_axis_handler(uint8_t axis);
extern uint8_t sys_id_wave_running(void);

extern void sys_id_wave_frequency_hz_set(float frequency_hz_set);
extern void sys_id_wave_lag_rad_set(float lag_rad_set);

// handlers for changing in GCS variables
extern void sys_id_wave_activate_handler(uint8_t activate);
extern void sys_id_wave_add_values(bool motors_on, bool override_on, pprz_t in_cmd[]);

#endif // SYS_ID_WAVE_H
