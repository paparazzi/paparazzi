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
 * @file "modules/system_identification/sys_id_wave.h"
 * @author Alessandro Collicelli
 * System excitation pure sine wave
 * 
 * This is the module implementation for a sine wave input. Use sys_id_wave by adding the module to your airframe file and
 * adding the following line to the top of the <command_laws> section of your airframe file:
 *
 * <call fun="sys_id_wave_add_values(autopilot_get_motors_on(),FALSE,values)"/>
 *
 * In the GCS you can then start and stop the wave, change wave frequency. 
 * Documentation of the specific options can be found in the module xml file.
 */

#include "std.h"
#include "math.h"
#include "pprz_wave.h"

#include "sys_id_wave.h"

#include "modules/datalink/telemetry.h"
#include "generated/airframe.h"
#include "mcu_periph/sys_time.h"
#include "filters/low_pass_filter.h"
#include "math/pprz_random.h"

#ifndef WAVE_AXES
#define WAVE_AXES {COMMAND_ROLL,COMMAND_PITCH,COMMAND_YAW,COMMAND_THRUST}
#endif

#ifndef WAVE_ENABLED
#define WAVE_ENABLED TRUE
#endif



static struct wave_t wave_s;
uint8_t wave_active = false;
uint8_t wave_axis = 0;
pprz_t wave_amplitude = 0;
float frequency_hz_ = 0.0f;
float lag_rad_ = 0.0f;

static const int8_t ACTIVE_WAVE_AXES[] = WAVE_AXES;
#define WAVE_NB_AXES sizeof ACTIVE_WAVE_AXES / sizeof ACTIVE_WAVE_AXES[0] // Number of items in ACTIVE_WAVE_AXES

// Chirp and noise values for all axes (indices correspond to the axes given in WAVE_AXES)
static pprz_t current_wave_values[WAVE_NB_AXES];

static void set_current_wave_values(void){
    if (wave_active){
        current_wave_values[wave_axis] = (int16_t)(wave_amplitude * wave_s.current_value);
        wave_s.is_running = true;
    }
    else{
        for (uint8_t i = 0; i < WAVE_NB_AXES; i++) {
            current_wave_values[i] = 0;
            wave_s.is_running = false;
        }
    }
}
static void send_wave(struct transport_tx *trans, struct link_device *dev){
    pprz_msg_send_WAVE(trans, dev, AC_ID, &wave_active, &wave_axis, &wave_amplitude, &wave_s.lag_rad, &wave_s.frequency_hz,
                       &current_wave_values[wave_axis]);
}

static void start_wave(void){
    wave_reset(&wave_s, get_sys_time_float());
    wave_active = true;
    set_current_wave_values();
    
}

static void stop_wave(void){
    wave_reset(&wave_s, get_sys_time_float());
    wave_active = false;
    set_current_wave_values();
    
}

void sys_id_wave_activate_handler(uint8_t activate){
    wave_active = activate;
    if (wave_active) {
        wave_init(&wave_s, get_sys_time_float(), get_sys_time_float(), frequency_hz_, lag_rad_);
        start_wave();
    } else {
        stop_wave();
    }
}

void sys_id_wave_axis_handler(uint8_t axis)
{
    if (axis < WAVE_NB_AXES) {
        wave_axis = axis;
    }
}


uint8_t sys_id_wave_running(void){
    return wave_active;
}


void sys_id_wave_frequency_hz_set(float frequency_hz_set)
{
        frequency_hz_ = frequency_hz_set;
        wave_init(&wave_s, get_sys_time_float(), get_sys_time_float(), frequency_hz_, lag_rad_);
}

void sys_id_wave_lag_rad_set(float lag_rad_set)
{
        lag_rad_ = lag_rad_set;
        wave_init(&wave_s, get_sys_time_float(), get_sys_time_float(), frequency_hz_, lag_rad_);
}


void sys_id_wave_init(void)
{
    wave_init(&wave_s, get_sys_time_float(), get_sys_time_float(), frequency_hz_, lag_rad_);
    set_current_wave_values();
    register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_WAVE, send_wave);
    
}

void sys_id_wave_run(void)
{
// #if WAVE_ENABLED

    if (wave_active) {
        if (!wave_is_running(&wave_s)) {
            stop_wave();
        } else {
            wave_update(&wave_s, get_sys_time_float());
            set_current_wave_values();
        }
    }
// #endif
}

void sys_id_wave_add_values(bool motors_on, bool override_on, pprz_t in_cmd[])
{
    (void)(override_on); // Suppress unused parameter warnings

#if WAVE_ENABLED

if (motors_on) {
    for (uint8_t i = 0; i < WAVE_NB_AXES; i++) {
        in_cmd[ACTIVE_WAVE_AXES[i]] += current_wave_values[i];
        BoundAbs(in_cmd[ACTIVE_WAVE_AXES[i]], MAX_PPRZ);
    }
}

#endif
}
