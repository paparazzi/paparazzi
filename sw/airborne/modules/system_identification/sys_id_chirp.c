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
 * @file "modules/system_identification/sys_id_chirp.c"
 * @author Joost Meulenbeld
 * System identification chirp
 */

#include "std.h"

#include "sys_id_chirp.h"
#include "pprz_chirp.h"

#include "modules/datalink/telemetry.h"
#include "generated/airframe.h"
#include "mcu_periph/sys_time.h"
#include "filters/low_pass_filter.h"
#include "math/pprz_random.h"


#ifndef SYS_ID_CHIRP_AXES
#define SYS_ID_CHIRP_AXES {COMMAND_ROLL,COMMAND_PITCH,COMMAND_YAW,COMMAND_THRUST}
#endif

#ifndef SYS_ID_CHIRP_ENABLED
#define SYS_ID_CHIRP_ENABLED TRUE
#endif

#ifndef SYS_ID_CHIRP_USE_NOISE
#define SYS_ID_CHIRP_USE_NOISE TRUE
#endif

#ifdef SYS_ID_CHIRP_RADIO_CHANNEL
#include "modules/radio_control/radio_control.h"
pprz_t previous_radio_value_chirp = 0;
#endif

// #ifndef SYS_ID_CHIRP_EXPONENTIAL
// #define SYS_ID_CHIRP_EXPONENTIAL TRUE
// #endif

// #ifndef SYS_ID_CHIRP_FADEIN
// #define SYS_ID_CHIRP_FADEIN TRUE
// #endif


static struct chirp_t chirp;
uint8_t chirp_active = FALSE;
uint8_t chirp_axis = 0;
pprz_t chirp_amplitude = 0;
float chirp_noise_stdv_onaxis_ratio = 0.1;
float chirp_noise_stdv_offaxis = 200;
float chirp_fstart_hz = 1.0f;
float chirp_fstop_hz = 5.0f;
float chirp_length_s = 20;

uint8_t chirp_fade_in = false;
uint8_t chirp_exponential = false;

// The axes on which noise and chirp values can be applied
static const int8_t SYS_ID_ACTIVE_CHIRP_AXES[] = SYS_ID_CHIRP_AXES;
#define SYS_ID_CHIRP_NB_AXES sizeof SYS_ID_ACTIVE_CHIRP_AXES / sizeof SYS_ID_ACTIVE_CHIRP_AXES[0] // Number of items in ACTIVE_CHIRP_AXES

// Filters used to cut-off the gaussian noise fed into the actuator channels
static struct FirstOrderLowPass filters[SYS_ID_CHIRP_NB_AXES];

// Chirp and noise values for all axes (indices correspond to the axes given in CHIRP_AXES)
static pprz_t current_chirp_values[SYS_ID_CHIRP_NB_AXES];

static void set_current_chirp_values(void)
{
    // initializing at zero the chirp input for every axis
    for (uint8_t i = 0; i < SYS_ID_CHIRP_NB_AXES; i++) {
      current_chirp_values[i] = 0;
    }  
    // adding values if the chirp is active
    if (chirp_active) {
      // adding extra  on the chirp signal (both on-axis and off axis)
      #if SYS_ID_CHIRP_USE_NOISE

          float amplitude, noise;
          for (uint8_t i = 0; i < SYS_ID_CHIRP_NB_AXES; i++) {
            noise = update_first_order_low_pass(&filters[i], rand_gaussian());
            amplitude = chirp_axis == i ? chirp_noise_stdv_onaxis_ratio * chirp_amplitude : chirp_noise_stdv_offaxis;
            current_chirp_values[i] += (int32_t)(noise * amplitude);
          }

      #endif
      // adding nominal chirp value
      current_chirp_values[chirp_axis] += (int32_t)(chirp_amplitude * chirp.current_value);
    } else {
      for (uint8_t i = 0; i < SYS_ID_CHIRP_NB_AXES; i++) {
        current_chirp_values[i] = 0;
      }
  }
}

static void send_chirp(struct transport_tx *trans, struct link_device *dev)
{
  pprz_msg_send_CHIRP(trans, dev, AC_ID, &chirp_active, &chirp.percentage_done, &chirp.current_frequency_hz,
                      &chirp_axis, &chirp_amplitude, &chirp_fstart_hz, &chirp_fstop_hz, &chirp_noise_stdv_onaxis_ratio,
                      &chirp_noise_stdv_offaxis, &current_chirp_values[chirp_axis], &chirp_fade_in, &chirp_exponential);

}

static void start_chirp(void)
{
  chirp_reset(&chirp, get_sys_time_float());
  chirp_active = TRUE;
  set_current_chirp_values();
}

static void stop_chirp(void)
{
  chirp_reset(&chirp, get_sys_time_float());
  chirp_active = FALSE;
  set_current_chirp_values();
}

void sys_id_chirp_activate_handler(uint8_t activate)
{
  chirp_active = activate;
  #ifdef SYS_ID_CHIRP_RADIO_CHANNEL
    // Don't activate chirp when radio signal is low
    if (radio_control.values[SYS_ID_CHIRP_RADIO_CHANNEL] < 1750)
    {
        chirp_active = 0;
    }
  #endif
  if (chirp_active) {
    chirp_init(&chirp, chirp_fstart_hz, chirp_fstop_hz, chirp_length_s, get_sys_time_float(), chirp_exponential,
               chirp_fade_in);
    start_chirp();
  } else {
    stop_chirp();
  }
}

uint8_t sys_id_chirp_running(void)
{
  return chirp_active;
}

extern void sys_id_chirp_axis_handler(uint8_t axis)
{
  if (axis < SYS_ID_CHIRP_NB_AXES) {
    chirp_axis = axis;
  }
}

extern void sys_id_chirp_fstart_handler(float fstart)
{
  if (fstart < chirp_fstop_hz) {
    chirp_fstart_hz = fstart;
  }
}

extern void sys_id_chirp_fstop_handler(float fstop)
{
  if (fstop > chirp_fstart_hz) {
    chirp_fstop_hz = fstop;
  }
}

extern void sys_id_chirp_fade_in_activate_handler(uint8_t fade_in)
{
  chirp_fade_in = fade_in;
}

extern void sys_id_chirp_exponential_activate_handler(uint8_t exponential)
{
  chirp_exponential = exponential;
}

void sys_id_chirp_init(void)
{
#if SYS_ID_CHIRP_USE_NOISE

  init_random();

#endif

  chirp_init(&chirp, chirp_fstart_hz, chirp_fstop_hz, chirp_length_s, get_sys_time_float(), chirp_exponential,
             chirp_fade_in);
  set_current_chirp_values();
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_CHIRP, send_chirp);

  // Filter cutoff frequency is the chirp maximum frequency
  float tau = 1 / (chirp_fstop_hz * 2 * M_PI);
  for (uint8_t i = 0; i < SYS_ID_CHIRP_NB_AXES; i++) {
    init_first_order_low_pass(&filters[i], tau, SYS_ID_CHIRP_RUN_PERIOD, 0);
    current_chirp_values[i] = 0;
  }
}

void sys_id_chirp_run(void)
{
#if SYS_ID_CHIRP_ENABLED

  #ifdef SYS_ID_CHIRP_RADIO_CHANNEL
    // Check if chirp switched on when off before
    if (previous_radio_value_chirp < 1750)
    {
        if (radio_control.values[SYS_ID_CHIRP_RADIO_CHANNEL] > 1750)
        {
            // Activate chirp
            sys_id_chirp_activate_handler(1);
        }
    }
    // Check if chirp switched off when on before
    if (previous_radio_value_chirp > 1750)
    {
        if (radio_control.values[SYS_ID_CHIRP_RADIO_CHANNEL] < 1750)
        {
            // Deactivate chirp
            sys_id_chirp_activate_handler(0);
        }
    }
    previous_radio_value_chirp = radio_control.values[SYS_ID_CHIRP_RADIO_CHANNEL];
  #endif

  if (chirp_active) {
    if (!chirp_is_running(&chirp, get_sys_time_float())) {
      stop_chirp();
    } else {
      chirp_update(&chirp, get_sys_time_float());
      set_current_chirp_values();
    }
  }

#endif
}

void sys_id_chirp_add_values(bool UNUSED motors_on, bool UNUSED override_on, pprz_t UNUSED in_cmd[])
{

#if SYS_ID_CHIRP_ENABLED

  if (motors_on) {
    for (uint8_t i = 0; i < SYS_ID_CHIRP_NB_AXES; i++) {
      in_cmd[SYS_ID_ACTIVE_CHIRP_AXES[i]] += current_chirp_values[i];
      BoundAbs(in_cmd[SYS_ID_ACTIVE_CHIRP_AXES[i]], MAX_PPRZ);
    }
  }

#endif
}
