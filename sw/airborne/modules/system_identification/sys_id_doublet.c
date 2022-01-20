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
 *
 */
#include "std.h"

#include "sys_id_doublet.h"
#include "pprz_doublet.h"

#include "modules/datalink/telemetry.h"
#include "generated/airframe.h"
#include "mcu_periph/sys_time.h"

#ifndef DOUBLET_AXES
#define DOUBLET_AXES {COMMAND_ROLL,COMMAND_PITCH,COMMAND_YAW,COMMAND_THRUST}
#endif

#ifndef DOUBLET_ENABLED
#define DOUBLET_ENABLED TRUE
#endif



static struct doublet_t doublet;

uint8_t doublet_active = false;
uint8_t doublet_mode_3211 = false;

uint8_t doublet_axis = 0;

pprz_t doublet_amplitude = 4500;
float doublet_length_s = 20.0f;
float doublet_extra_waiting_time_s = 0.0f;



static const int8_t ACTIVE_DOUBLET_AXES[] = DOUBLET_AXES;
#define DOUBLET_NB_AXES sizeof ACTIVE_DOUBLET_AXES / sizeof ACTIVE_DOUBLET_AXES[0] // Number of items in ACTIVE_DOUBLET_AXES

static pprz_t current_doublet_values[DOUBLET_NB_AXES];

static void set_current_doublet_values(void)
{
    if (doublet_active) {
        current_doublet_values[doublet_axis] = (int32_t)(doublet_amplitude * doublet.current_value);
        
    } else {
        for (uint8_t i = 0; i < DOUBLET_NB_AXES; i++) {
            current_doublet_values[i] = 0;
        }
    }
}

static void send_doublet(struct transport_tx *trans, struct link_device *dev){
    pprz_msg_send_DOUBLET(trans, dev, AC_ID, &doublet_active,
                        &doublet_axis, &doublet_amplitude,
                        &current_doublet_values[doublet_axis], &doublet_mode_3211);
}

static void start_doublet(void)
{
    doublet_reset(&doublet, get_sys_time_float());
    doublet_active = true;
    set_current_doublet_values();
}

static void stop_doublet(void)
{
    doublet_reset(&doublet, get_sys_time_float());
    doublet_active = false;
    set_current_doublet_values();
}

uint8_t sys_id_doublet_running(void){
    return doublet_active;
}

void sys_id_doublet_activate_handler(uint8_t activate)
{
    doublet_active = activate;
    if (doublet_active) {
        doublet_init(&doublet, doublet_length_s, doublet_extra_waiting_time_s, get_sys_time_float(), doublet_mode_3211);
        start_doublet();
    } else {
        stop_doublet();
    }
}

void sys_id_doublet_axis_handler(uint8_t axis)
{
    if (axis < DOUBLET_NB_AXES) {
        doublet_axis = axis;
    }
}

void sys_id_doublet_mod3211_handler(uint8_t mode){
    doublet_mode_3211 = mode;
}

void sys_id_doublet_init(void)
{
    doublet_init(&doublet, doublet_length_s, doublet_extra_waiting_time_s, get_sys_time_float(), doublet_mode_3211);

    set_current_doublet_values();
    register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_DOUBLET, send_doublet);
    
    for (uint8_t i = 0; i < DOUBLET_NB_AXES; i++) {
        current_doublet_values[i] = 0;
    }
}

void sys_id_doublet_run(void)
{
    if (doublet_active) {
        if (!doublet_is_running(&doublet, get_sys_time_float())) {
            stop_doublet();
        } else {
            doublet_update(&doublet, get_sys_time_float());
            set_current_doublet_values();
        }
    }
    

}

void sys_id_doublet_add_values(bool motors_on, bool override_on, pprz_t in_cmd[])
{
    (void)(override_on); // Suppress unused parameter warnings

#if DOUBLET_ENABLED

if (motors_on) {
    for (uint8_t i = 0; i < DOUBLET_NB_AXES; i++) {
        in_cmd[ACTIVE_DOUBLET_AXES[i]] += current_doublet_values[i];
        BoundAbs(in_cmd[ACTIVE_DOUBLET_AXES[i]], MAX_PPRZ);
    }
}

#endif
}
