/*
 * Copyright (C) 2016 Gautier Hattenberger <gautier.hattenberger@enac.fr>
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
 * @file firmwares/rotorcraft/autopilot_utils.c
 *
 * Utility functions and includes for autopilots
 *
 */

#include "firmwares/rotorcraft/autopilot_utils.h"
#include "autopilot.h"
#include "firmwares/rotorcraft/autopilot_rc_helpers.h"
#include "state.h"
#include "modules/radio_control/radio_control.h"

/** Display descent speed in failsafe mode if needed */
PRINT_CONFIG_VAR(FAILSAFE_DESCENT_SPEED)


// Utility functions
#ifndef AUTOPILOT_DISABLE_AHRS_KILL
bool ap_ahrs_is_aligned(void)
{
  return stateIsAttitudeValid();
}
#else
PRINT_CONFIG_MSG("Using AUTOPILOT_DISABLE_AHRS_KILL")
bool ap_ahrs_is_aligned(void)
{
  return true;
}
#endif

#if defined RADIO_MODE_2x3

#define THRESHOLD_1d3_PPRZ (MAX_PPRZ / 3)
#define THRESHOLD_2d3_PPRZ ((MAX_PPRZ / 3) * 2)
/** Get autopilot mode as set by a RADIO_MODE 3-way switch and a 2-way switch, which are mixed together
 *  The 2 way switch negates the value, the 3 way switch changes in three steps from 0 - MAX_PPRZ.
 *  E.g. SW_1 has two positions (On/Off), SW_Mode has three positions (M1/M2/M3)
 *   1  Mode value
 *   Off  M1  -9500
 *   Off  M2  -4800
 *   Off  M3  -1850
 *   On M1  2100
 *   On M2  4900
 *   On M3  9600
 *  This function filters out the effect of SW_1, such that a normal 3-way switch comes out.
**/
uint8_t ap_mode_of_3x2way_switch(void)
{
  int val = radio_control.values[RADIO_MODE];
  if (radio_control.values[RADIO_MODE] < 0) {
    val = MAX_PPRZ + val;
  }
  if (val < THRESHOLD_1d3_PPRZ) {
    return MODE_MANUAL;
  } else if (val < THRESHOLD_2d3_PPRZ) {
    return MODE_AUTO1;
  } else {
    return autopilot_mode_auto2;
  }
}

#else

#define THRESHOLD_1_PPRZ (MIN_PPRZ / 2)
#define THRESHOLD_2_PPRZ (MAX_PPRZ / 2)

/** get autopilot mode as set by RADIO_MODE 3-way switch */
uint8_t ap_mode_of_3way_switch(void)
{
  if (radio_control.values[RADIO_MODE] > THRESHOLD_2_PPRZ) {
    return autopilot_mode_auto2;
  } else if (radio_control.values[RADIO_MODE] > THRESHOLD_1_PPRZ) {
    return MODE_AUTO1;
  } else {
    return MODE_MANUAL;
  }
}

#endif

/**
 * Get autopilot mode from two 2way switches.
 * RADIO_MODE switch just selectes between MANUAL and AUTO.
 * If not MANUAL, the RADIO_AUTO_MODE switch selects between AUTO1 and AUTO2.
 *
 * This is mainly a cludge for entry level radios with no three-way switch,
 * but two available two-way switches which can be used.
 */
#if defined RADIO_AUTO_MODE || defined(__DOXYGEN__)
uint8_t ap_mode_of_two_switches(void)
{
  if (radio_control.values[RADIO_MODE] < THRESHOLD_1_PPRZ) {
    /* RADIO_MODE in MANUAL position */
    return MODE_MANUAL;
  } else {
    /* RADIO_MODE not in MANUAL position.
     * Select AUTO mode bassed on RADIO_AUTO_MODE channel
     */
    if (radio_control.values[RADIO_AUTO_MODE] > THRESHOLD_2_PPRZ) {
      return autopilot_mode_auto2;
    } else {
      return MODE_AUTO1;
    }
  }
}
#endif


/** Set Rotorcraft commands.
 *  Limit thrust and/or yaw depending of the in_flight
 *  and motors_on flag status
 */
void WEAK set_rotorcraft_commands(pprz_t *cmd_out, int32_t *cmd_in, bool in_flight __attribute__((unused)), bool motors_on __attribute__((unused)))
{
#if !ROTORCRAFT_IS_HELI
#if !ROTORCRAFT_COMMANDS_YAW_ALWAYS_ENABLED
  if (!in_flight) {
    cmd_in[COMMAND_YAW] = 0;
  }
#endif
  if (!motors_on) {
    cmd_in[COMMAND_THRUST] = 0;
  }
#endif
  cmd_out[COMMAND_ROLL] = cmd_in[COMMAND_ROLL];
  cmd_out[COMMAND_PITCH] = cmd_in[COMMAND_PITCH];
  cmd_out[COMMAND_YAW] = cmd_in[COMMAND_YAW];
  cmd_out[COMMAND_THRUST] = cmd_in[COMMAND_THRUST];
}

