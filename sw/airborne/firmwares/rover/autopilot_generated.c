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
 * @file firmwares/rover/autopilot_generated.c
 *
 * Generated autopilot implementation.
 *
 */
#define AUTOPILOT_CORE_AP_C

#include "firmwares/rotorcraft/autopilot_generated.h"
#include "autopilot.h"

#include "modules/radio_control/radio_control.h"
#include "modules/core/commands.h"
#include "modules/actuators/actuators.h"
#include "modules/core/settings.h"
#include "modules/datalink/telemetry.h"

#include "generated/settings.h"


void autopilot_generated_init(void)
{
  // call generated init
  autopilot_core_ap_init();
  // copy generated mode to public mode (set at startup)
  autopilot.mode = autopilot_mode_ap;
}


void autopilot_generated_periodic(void)
{

  autopilot_core_ap_periodic_task();

  // copy generated mode to public mode (may be changed on internal exceptions)
  autopilot.mode = autopilot_mode_ap;

  /* Reset ground detection _after_ running flight plan
   */
  if (!autopilot_in_flight()) {
    autopilot.ground_detected = false;
    autopilot.detect_ground_once = false;
  }

}

/** AP mode setting handler
 *
 * FIXME generated something else for this?
 */
void autopilot_generated_SetModeHandler(float mode)
{
  autopilot_generated_set_mode(mode);
}


void autopilot_generated_set_mode(uint8_t new_autopilot_mode)
{
  autopilot_core_ap_set_mode(new_autopilot_mode);
  // copy generated mode to public mode
  autopilot.mode = autopilot_mode_ap;
}


void autopilot_generated_set_motors_on(bool motors_on)
{
  if (ap_ahrs_is_aligned() && motors_on
#ifdef AP_MODE_KILL
      && autopilot.mode != AP_MODE_KILL
#endif
      ) {
    autopilot.motors_on = true;
  } else {
    autopilot.motors_on = false;
  }
}


void autopilot_generated_on_rc_frame(void)
{
}
