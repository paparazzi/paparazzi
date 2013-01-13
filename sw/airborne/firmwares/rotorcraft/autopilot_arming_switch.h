/*
 * Copyright (C) 2012 The Paparazzi Team
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
 * @file firmwares/rotorcraft/autopilot_arming_switch.h
 *
 * Arm the motors using a switch.
 *
 */

#ifndef AUTOPILOT_ARMING_SWITCH_H
#define AUTOPILOT_ARMING_SWITCH_H

#include "autopilot_rc_helpers.h"

#ifndef RADIO_KILL_SWITCH
#error "You need to have a RADIO_KILL_SWITCH configured to arm the motors with the switch!"
#endif

enum arming_state {
  STATE_UNINIT,
  STATE_WAITING,
  STATE_STARTABLE,
  STATE_MOTORS_ON
};

enum arming_state autopilot_arming_state;
bool_t autopilot_unarmed_in_auto;

static inline void autopilot_arming_init(void) {
  autopilot_arming_state = STATE_UNINIT;
  autopilot_unarmed_in_auto = FALSE;
}

static inline void autopilot_arming_set(bool_t motors_on) {
  if (motors_on) {
    autopilot_arming_state = STATE_MOTORS_ON;
  }
  else {
    if (autopilot_arming_state == STATE_MOTORS_ON) {
      autopilot_arming_state = STATE_STARTABLE;
      /* if turned off in an AUTO mode, remember it so it can be turned on again in AUTO */
      if (autopilot_mode != MODE_MANUAL) {
        autopilot_unarmed_in_auto = TRUE;
      }
      else {
        autopilot_unarmed_in_auto = FALSE;
      }
    }
  }
}

/**
 * State machine to check if motors should be turned ON or OFF using the kill switch.
 * If kill switch is off during startup (unkilled), you need to kill again first,
 * then unkill to start.
 * Also to start the motors, throttle needs to be down, other sticks centered,
 * AHRS aligned and you need be in manual mode.
 */
static inline void autopilot_arming_check_motors_on( void ) {
  switch(autopilot_arming_state) {
  case STATE_UNINIT:
    autopilot_motors_on = FALSE;
    if (kill_switch_is_on()) {
      autopilot_arming_state = STATE_STARTABLE;
    }
    else {
      autopilot_arming_state = STATE_WAITING;
    }
    break;
  case STATE_WAITING:
    autopilot_motors_on = FALSE;
    if (kill_switch_is_on()) {
      autopilot_arming_state = STATE_STARTABLE;
    }
    break;
  case STATE_STARTABLE:
    autopilot_motors_on = FALSE;
    if (!kill_switch_is_on() &&
        THROTTLE_STICK_DOWN() &&
        rc_attitude_sticks_centered() &&
        (autopilot_mode == MODE_MANUAL || autopilot_unarmed_in_auto)) {
      autopilot_arming_state = STATE_MOTORS_ON;
    }
    break;
  case STATE_MOTORS_ON:
    autopilot_motors_on = TRUE;
    if (kill_switch_is_on()) {
      /* if killed, go to STATE_STARTABLE where motors will be turned off */
      autopilot_arming_state = STATE_STARTABLE;
      /* if turned off in an AUTO mode, remember it so it can be turned on again in AUTO */
      if (autopilot_mode != MODE_MANUAL) {
        autopilot_unarmed_in_auto = TRUE;
      }
      else {
        autopilot_unarmed_in_auto = FALSE;
      }
    }
    break;
  default:
    break;
  }

}

#endif /* AUTOPILOT_ARMING_SWITCH_H */
