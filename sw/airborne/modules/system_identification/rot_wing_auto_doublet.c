/*
 * Copyright (C) 2023 Dennis van Wijngaarden <D.C.vanWijngaarden@tudelft.nl>
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

/** @file "modules/system_identification/rot_wing_auto_doublet.c"
 * @author Dennis van Wijngaarden <D.C.vanWijngaarden@tudelft.nl>
 * Module that automatically runs a doublet program for the rotating wing drone
 */
#include "std.h"

#include "modules/system_identification/rot_wing_auto_doublet.h"
#include "modules/system_identification/sys_id_doublet.h"
#include "modules/system_identification/pprz_doublet.h"

#include "mcu_periph/sys_time.h"

// Perform checks if mandatory macros are defined

#ifndef ROT_WING_AUTO_DOUBLET_N_ACTUATORS
#error No doublet actuators ROT_WING_AUTO_DOUBLET_N_ACTUATORS defined
#endif

#ifndef ROT_WING_AUTO_DOUBLET_ACTUATORS
#error No doublet actuators ROT_WING_AUTO_DOUBLET_ACTUATORS defined
#endif

#ifndef ROT_WING_AUTO_DOUBLET_AMPLITUDE
#error No doublet actuators ROT_WING_AUTO_DOUBLET_AMPLITUDE defined
#endif

uint8_t rot_wing_auto_doublet_actuators[ROT_WING_AUTO_DOUBLET_N_ACTUATORS] = ROT_WING_AUTO_DOUBLET_ACTUATORS;
int16_t rot_wing_auto_doublet_amplitude[ROT_WING_AUTO_DOUBLET_N_ACTUATORS] = ROT_WING_AUTO_DOUBLET_AMPLITUDE;

float rot_wing_auto_doublet_time = 0.5;         // time of one doublet
float rot_wing_auto_doublet_interval_time = 5.; // time interval for doublets
int8_t rot_wing_auto_doublet_n_repeat = 5;      // The number of times a doublet has to be repeated on a single actuator

bool rot_wing_auto_doublet_activated = false;

// Other variables needed for the periodic loop
uint8_t rot_wing_auto_doublet_n_doublets;

float rot_wing_auto_doublet_start_time_s;
float rot_wing_auto_doublet_timer;
uint8_t rot_wing_auto_doublet_counter;

inline void perform_rot_wing_auto_doublet(uint8_t actuator_index);
inline void rot_wing_auto_doublet_on_deactivation(void);

void init_rot_wing_auto_doublet(void)
{
  rot_wing_auto_doublet_n_doublets = ROT_WING_AUTO_DOUBLET_N_ACTUATORS * rot_wing_auto_doublet_n_repeat;
  rot_wing_auto_doublet_start_time_s = 0.;
  rot_wing_auto_doublet_timer = 0.;
  rot_wing_auto_doublet_counter = 0;
}

void periodic_rot_wing_auto_doublet(void)
{
  // your periodic code here.
  // freq = 20.0 Hz

  // Run if doublet activated
  if (rot_wing_auto_doublet_activated)
  {
    float current_time_s = get_sys_time_float();
    rot_wing_auto_doublet_timer = current_time_s - rot_wing_auto_doublet_start_time_s;

    // Check when to increase the counters and perform doublet
    uint8_t current_index = rot_wing_auto_doublet_timer / rot_wing_auto_doublet_interval_time;

    // stop doublets when all doublets done
    if (current_index >= rot_wing_auto_doublet_n_doublets)
    {
      rot_wing_auto_doublet_on_deactivation();
    }
    else
    { // continue doublets when not finished yet
      // Check if doublet should be performed and index inxreased
      if (current_index > rot_wing_auto_doublet_counter)
      {
        uint8_t actuator_index = current_index / rot_wing_auto_doublet_n_repeat;
        perform_rot_wing_auto_doublet(actuator_index);
        rot_wing_auto_doublet_counter++;
      }
    }
  }
}

void perform_rot_wing_auto_doublet(uint8_t actuator_index)
{
  // Do nothing id doublet index is invalid
  if (actuator_index >= ROT_WING_AUTO_DOUBLET_N_ACTUATORS)
  {
    return;
  }
  // Set correct settings
  sys_id_doublet_axis_handler(rot_wing_auto_doublet_actuators[actuator_index]);
  doublet_length_s = rot_wing_auto_doublet_time;
  doublet_amplitude = rot_wing_auto_doublet_amplitude[actuator_index];

  // call doublet activation handler
  sys_id_doublet_activate_handler(1);
}

void rot_wing_auto_doublet_on_activation(uint8_t active)
{
  rot_wing_auto_doublet_activated = active;
  // If the activation boolean is set to true
  if (rot_wing_auto_doublet_activated)
  {
    if (rot_wing_auto_doublet_timer > 0)
    {
      // Do nothing if already activated
    }
    else
    {
      // Activate auto doublet if not activated yet
      rot_wing_auto_doublet_start_time_s = get_sys_time_float();
      rot_wing_auto_doublet_timer = 0;
      perform_rot_wing_auto_doublet(0);
    }
  }
  else
  { // If the activation boolean is set to false
    rot_wing_auto_doublet_on_deactivation();
  }
}

void rot_wing_auto_doublet_on_deactivation(void)
{
  // call doublet deactivation handler
  sys_id_doublet_activate_handler(0);
  rot_wing_auto_doublet_activated = false;
  rot_wing_auto_doublet_timer = 0;
  rot_wing_auto_doublet_counter = 0;
  doublet_amplitude = 0;
}