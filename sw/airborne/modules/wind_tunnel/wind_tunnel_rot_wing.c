/*
 * Copyright (C) 2022 Dennis van Wijngaarden <D.C.vanWijngaarden@tudelft.nl>
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

/** @file "modules/wind_tunnel/wind_tunnel_rot_wing.c"
 * @author Dennis van Wijngaarden <D.C.vanWijngaarden@tudelft.nl>
 * This module allows the user to control seperate actuators for example during wind tunnel experiments.
 */

#include "modules/wind_tunnel/wind_tunnel_rot_wing.h"
#include "mcu_periph/sys_time.h"

int16_t actuators_wt[11] = {0,0,0,0,0,0,0,0,0,0,0};
int16_t actuators_slider_wt[11] = {0,0,0,0,0,0,0,0,0,0,0};
int16_t motors_slider_wt = 0;

bool actuator_is_servo_wt[11] = {0,0,0,0,0,1,1,1,1,1,1};

bool motors_on_wt = false;
bool motor_on_wt[5] = {1,1,1,1,1};

// actuator sweep parameters
uint8_t wt_actuator_sweep_index = 0;
int16_t wt_input_min_cmd = 0;
int16_t wt_input_max_cmd = 0;
float wt_input_steptime = 5;

struct wt_active_sweep_params {
  uint8_t sweep_index;
  int16_t min_cmd;
  int16_t max_cmd;
  float steptime;
  float total_time;
} wt_active_sweep_p;

struct wt_active_motors_sweep_params {
  int16_t begin_cmd[4];
  int16_t end_cmd;
  float steptime;
} wt_active_motors_sweep_p;

// status indicators
bool wt_sweep_running = false;
bool wt_sweep_motors_running = false;
float wt_sweep_timer_start = 0;
float wt_sweep_motors_timer_start = 0;

void init_wt_rot_wing(void)
{
  // your init code here
  wt_active_sweep_p.sweep_index = wt_actuator_sweep_index;
  wt_active_sweep_p.min_cmd     = wt_input_min_cmd;
  wt_active_sweep_p.max_cmd     = wt_input_max_cmd;
  wt_active_sweep_p.steptime    = wt_input_steptime;
  wt_active_sweep_p.total_time  = wt_active_sweep_p.steptime * 4;

  for (uint8_t i = 0; i<4; i++)
  {
    wt_active_motors_sweep_p.begin_cmd[i] = actuators_wt[i];
  }
  wt_active_motors_sweep_p.end_cmd = motors_slider_wt;
  wt_active_motors_sweep_p.steptime = wt_input_steptime;
}

void event_wt_rot_wing(void)
{
  int16_t actuators_temp[11] = {0,0,0,0,0,0,0,0,0,0,0}; 

  // Put prefered actuator commands
  for (uint8_t i = 0; i<11; i++)
  {
    actuators_temp[i] = actuators_slider_wt[i];
  }

  // If sweep is running, put current sweep value on actuator
  if (wt_sweep_running) {
    float sweep_time = get_sys_time_float() - wt_sweep_timer_start;
    if (sweep_time > wt_active_sweep_p.total_time)
    {
      wt_sweep_running = false;
    } else {
      float progress = sweep_time / wt_active_sweep_p.steptime; // If progress > 4, sweep done
      int16_t sweep_cmd = 0;
      float stage_progress;
      if (progress < 1) {
        stage_progress = progress;
        sweep_cmd = actuators_slider_wt[wt_active_sweep_p.sweep_index] + (int16_t)((float)(wt_active_sweep_p.min_cmd - actuators_slider_wt[wt_active_sweep_p.sweep_index]) * stage_progress);
      } else if (progress < 2) {
        stage_progress = progress - 1.;
        sweep_cmd = wt_active_sweep_p.min_cmd + (int16_t)((float)(actuators_slider_wt[wt_active_sweep_p.sweep_index] - wt_active_sweep_p.min_cmd) * stage_progress);
      } else if (progress < 3) {
        stage_progress = progress - 2.;
        sweep_cmd = actuators_slider_wt[wt_active_sweep_p.sweep_index] + (int16_t)((float)(wt_active_sweep_p.max_cmd - actuators_slider_wt[wt_active_sweep_p.sweep_index]) * stage_progress);
      } else if (progress < 4) {
        stage_progress = progress - 3.;
        sweep_cmd = wt_active_sweep_p.max_cmd + (int16_t)((float)(actuators_slider_wt[wt_active_sweep_p.sweep_index] - wt_active_sweep_p.max_cmd) * stage_progress);
      }

      
      actuators_temp[wt_active_sweep_p.sweep_index] = sweep_cmd;
    }
  }

  // If motors sweep running
  if (wt_sweep_motors_running) {
    float sweep_time = get_sys_time_float() - wt_sweep_motors_timer_start;
    if (sweep_time > wt_active_motors_sweep_p.steptime) {
      wt_sweep_motors_running = false;
    } else {
      float progress = sweep_time / wt_active_motors_sweep_p.steptime;
      for (uint8_t i = 0; i<4; i++) {
        actuators_temp[i] = wt_active_motors_sweep_p.begin_cmd[i] + (int16_t)((float)(wt_active_motors_sweep_p.end_cmd - wt_active_motors_sweep_p.begin_cmd[i]) * progress);
      }
    }
  }

  // Evaluate motor off
  for (uint8_t i = 0; i < 5; i++)
  {
    if (!motor_on_wt[i] || !motors_on_wt)
    {
      actuators_temp[i] = -9600;
    }
  }

  // Bound actuators_temp and copy to actuators list
  for (uint8_t i = 0; i < 11; i++)
  {
    Bound(actuators_temp[i], -9600, 9600);
    actuators_wt[i] = actuators_temp[i];
  }
}

void evaluate_motor_commands(void)
{
  for (uint8_t i = 0; i < 5; i++)
  {
    if (!motor_on_wt[i] || !motors_on_wt)
    {
      actuators_wt[i] = -9600;
    }
  }
}

void wind_tunnel_rot_wing_sweep_handler(bool activate)
{
  // Only start when there is a non active sweep
  if (!wt_sweep_running && !wt_sweep_motors_running && activate)
  {
    // Copy sweep values to active sweep values
    wt_active_sweep_p.sweep_index = wt_actuator_sweep_index;
    wt_active_sweep_p.min_cmd     = wt_input_min_cmd;
    wt_active_sweep_p.max_cmd     = wt_input_max_cmd;
    wt_active_sweep_p.steptime    = wt_input_steptime;
    wt_active_sweep_p.total_time  = wt_active_sweep_p.steptime * 4;
    wt_sweep_running = true;
    wt_sweep_timer_start = get_sys_time_float();
  } else if (!activate) {
    wt_sweep_running = false;
  }
}

void wind_tunnel_rot_wing_sweep_motors_handler(bool activate)
{
  // switch off sweep if running:
  wt_sweep_running = false;
  // Switch on motors sweep
  if (!wt_sweep_motors_running && activate)
  {
    for (uint8_t i = 0; i<4; i++)
    {
      wt_active_motors_sweep_p.begin_cmd[i] = actuators_wt[i];
    }
    wt_active_motors_sweep_p.end_cmd = motors_slider_wt;
    wt_active_motors_sweep_p.steptime = wt_input_steptime;
    wt_sweep_motors_timer_start = get_sys_time_float();
    wt_sweep_motors_running = true;

    // set sliders
    for (uint8_t i = 0; i<4; i++)
    {
      actuators_slider_wt[i] = wt_active_motors_sweep_p.end_cmd;
    }
  }
}