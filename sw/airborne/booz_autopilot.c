/*
 * $Id$
 *  
 * Copyright (C) 2008  Antoine Drouin
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
 *
 */

#include "booz_autopilot.h"

#include "radio_control.h"
#include "commands.h"
#include "booz_control.h"
#include "booz_nav.h"

uint8_t booz_autopilot_mode;

void booz_autopilot_init(void) {
  booz_autopilot_mode = BOOZ_AP_MODE_FAILSAFE;
}

void booz_autopilot_periodic_task(void) {

  switch (booz_autopilot_mode) {
  case BOOZ_AP_MODE_FAILSAFE:
  case BOOZ_AP_MODE_KILL:
    SetCommands(commands_failsafe);
    break;
  case BOOZ_AP_MODE_RATE:
    booz_control_rate_run();
    SetCommands(booz_control_commands);
    break;
  case BOOZ_AP_MODE_ATTITUDE:
  case BOOZ_AP_MODE_HEADING_HOLD:
    booz_control_attitude_run();
    SetCommands(booz_control_commands);
    break;
  case BOOZ_AP_MODE_NAV:
    booz_nav_run();
    booz_control_attitude_run();
    SetCommands(booz_control_commands);
    break;
  }

}


void booz_autopilot_on_rc_event(void) {
  /* I think this should be hidden in rc code */
  /* the ap gets a mode everytime - the rc filters it */
  if (rc_values_contains_avg_channels) {
    booz_autopilot_mode = BOOZ_AP_MODE_OF_PPRZ(rc_values[RADIO_MODE]);
    rc_values_contains_avg_channels = FALSE;
  }
  switch (booz_autopilot_mode) {
  case BOOZ_AP_MODE_RATE:
    booz_control_rate_read_setpoints_from_rc();
    break;
  case BOOZ_AP_MODE_ATTITUDE:
  case BOOZ_AP_MODE_HEADING_HOLD:
    booz_control_attitude_read_setpoints_from_rc();
    break;
  case BOOZ_AP_MODE_NAV:
    booz_nav_read_setpoints_from_rc();
    break;
  }
}
