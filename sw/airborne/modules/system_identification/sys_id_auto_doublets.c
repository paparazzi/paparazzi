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

/** @file "modules/system_identification/sys_id_auto_doublets.c"
 * @author Dennis van Wijngaarden <D.C.vanWijngaarden@tudelft.nl>
 * Module that automatically runs a doublet program for the rotating wing drone
 */
#include "std.h"

#include "modules/system_identification/sys_id_auto_doublets.h"
#include "modules/system_identification/sys_id_doublet.h"
#include "modules/system_identification/pprz_doublet.h"

#include "mcu_periph/sys_time.h"
#include "generated/airframe.h"

// Perform checks if mandatory macros are defined

#ifndef SYS_ID_AUTO_DOUBLETS_N_ACTUATORS
#error No doublet actuators SYS_ID_AUTO_DOUBLETS_N_ACTUATORS defined
#endif

#ifndef SYS_ID_AUTO_DOUBLETS_ACTUATORS
#error No doublet actuators SYS_ID_AUTO_DOUBLETS_ACTUATORS defined
#endif

#ifndef SYS_ID_AUTO_DOUBLETS_AMPLITUDE
#error No doublet actuators SYS_ID_AUTO_DOUBLETS_AMPLITUDE defined
#endif

uint8_t sys_id_auto_doublets_actuators[SYS_ID_AUTO_DOUBLETS_N_ACTUATORS] = SYS_ID_AUTO_DOUBLETS_ACTUATORS;
int16_t sys_id_auto_doublets_amplitude[SYS_ID_AUTO_DOUBLETS_N_ACTUATORS] = SYS_ID_AUTO_DOUBLETS_AMPLITUDE;

float sys_id_auto_doublets_time = 0.5;         // time of one doublet
float sys_id_auto_doublets_interval_time = 5.; // time interval for doublets
int8_t sys_id_auto_doublets_n_repeat = 5;      // The number of times a doublet has to be repeated on a single actuator

bool sys_id_auto_doublets_activated = false;

// Other variables needed for the periodic loop
uint8_t sys_id_auto_doublets_n_doublets;

float sys_id_auto_doublets_start_time_s;
float sys_id_auto_doublets_timer;
uint8_t sys_id_auto_doublets_counter;

inline void perform_sys_id_auto_doublets(uint8_t actuator_index);
inline void sys_id_auto_doublets_on_deactivation(void);

void init_sys_id_auto_doublets(void)
{
  sys_id_auto_doublets_n_doublets = SYS_ID_AUTO_DOUBLETS_N_ACTUATORS * sys_id_auto_doublets_n_repeat;
  sys_id_auto_doublets_start_time_s = 0.;
  sys_id_auto_doublets_timer = 0.;
  sys_id_auto_doublets_counter = 0;
}

void periodic_sys_id_auto_doublets(void)
{
  // your periodic code here.
  // freq = 20.0 Hz

  // Run if doublet activated
  if (sys_id_auto_doublets_activated)
  {
    float current_time_s = get_sys_time_float();
    sys_id_auto_doublets_timer = current_time_s - sys_id_auto_doublets_start_time_s;

    // Check when to increase the counters and perform doublet
    uint8_t current_index = sys_id_auto_doublets_timer / sys_id_auto_doublets_interval_time;

    // stop doublets when all doublets done
    if (current_index >= sys_id_auto_doublets_n_doublets)
    {
      sys_id_auto_doublets_on_deactivation();
    }
    else
    { // continue doublets when not finished yet
      // Check if doublet should be performed and index inxreased
      if (current_index > sys_id_auto_doublets_counter)
      {
        uint8_t actuator_index = current_index / sys_id_auto_doublets_n_repeat;
        perform_sys_id_auto_doublets(actuator_index);
        sys_id_auto_doublets_counter++;
      }
    }
  }
}

void perform_sys_id_auto_doublets(uint8_t actuator_index)
{
  // Do nothing id doublet index is invalid
  if (actuator_index >= SYS_ID_AUTO_DOUBLETS_N_ACTUATORS)
  {
    return;
  }
  // Set correct settings
  sys_id_doublet_axis_handler(sys_id_auto_doublets_actuators[actuator_index]);
  doublet_length_s = sys_id_auto_doublets_time;
  doublet_amplitude = sys_id_auto_doublets_amplitude[actuator_index];

  // call doublet activation handler
  sys_id_doublet_activate_handler(1);
}

void sys_id_auto_doublets_on_activation(uint8_t active)
{
  sys_id_auto_doublets_activated = active;
  // If the activation boolean is set to true
  if (sys_id_auto_doublets_activated)
  {
    if (sys_id_auto_doublets_timer > 0)
    {
      // Do nothing if already activated
    }
    else
    {
      // Activate auto doublet if not activated yet
      sys_id_auto_doublets_start_time_s = get_sys_time_float();
      sys_id_auto_doublets_timer = 0;
      perform_sys_id_auto_doublets(0);
    }
  }
  else
  { // If the activation boolean is set to false
    sys_id_auto_doublets_on_deactivation();
  }
}

void sys_id_auto_doublets_on_deactivation(void)
{
  // call doublet deactivation handler
  sys_id_doublet_activate_handler(0);
  sys_id_auto_doublets_activated = false;
  sys_id_auto_doublets_timer = 0;
  sys_id_auto_doublets_counter = 0;
  doublet_amplitude = 0;
}