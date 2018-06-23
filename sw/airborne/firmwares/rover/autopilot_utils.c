/*
 * Copyright (C) 2018 Gautier Hattenberger <gautier.hattenberger@enac.fr>
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
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */

/**
 * @file firmwares/rover/autopilot_utils.c
 *
 * Utility functions and includes for autopilots
 *
 */

#include "firmwares/rotorcraft/autopilot_utils.h"
#include "autopilot.h"
#include "firmwares/rotorcraft/autopilot_rc_helpers.h"
#include "state.h"
#include "subsystems/radio_control.h"

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

#define THRESHOLD_1_PPRZ (MIN_PPRZ / 2)
#define THRESHOLD_2_PPRZ (MAX_PPRZ / 2)

/** get autopilot mode as set by RADIO_MODE 3-way switch */
uint8_t ap_mode_of_3way_switch(void)
{
  if (radio_control.values[RADIO_MODE] > THRESHOLD_2_PPRZ) {
    return MODE_AUTO2;
  } else if (radio_control.values[RADIO_MODE] > THRESHOLD_1_PPRZ) {
    return MODE_AUTO1;
  } else {
    return MODE_MANUAL;
  }
}

