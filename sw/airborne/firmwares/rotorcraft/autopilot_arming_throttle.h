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
 * @file firmwares/rotorcraft/autopilot_arming_throttle.h
 *
 * Automatically arm the motors when applying throttle.
 *
 */

#ifndef AUTOPILOT_ARMING_THROTTLE_H
#define AUTOPILOT_ARMING_THROTTLE_H

#include "autopilot_rc_helpers.h"

#define AUTOPILOT_ARMING_DELAY 10

enum arming_throttle_state {
  STATE_UNINIT,
  STATE_WAITING,
  STATE_MOTORS_OFF_READY,
  STATE_ARMING,
  STATE_MOTORS_ON,
  STATE_UNARMING
};

enum arming_throttle_state autopilot_arming_state;
uint8_t autopilot_arming_delay_counter;
bool_t autopilot_unarmed_in_auto;

static inline void autopilot_arming_init(void) {
  autopilot_arming_state = STATE_UNINIT;
  autopilot_arming_delay_counter = 0;
  autopilot_unarmed_in_auto = FALSE;
}

static inline void autopilot_arming_set(bool_t motors_on) {
  if (motors_on) {
    autopilot_arming_state = STATE_MOTORS_ON;
  }
  else {
    if (autopilot_arming_state == STATE_MOTORS_ON) {
      autopilot_arming_state = STATE_WAITING;
    }
  }
}

/**
 * State machine to check if motors should be turned ON or OFF.
 * - automatically unkill when applying throttle
 * - if throttle was not down at startup, you need to put throttle down again first
 * - other sticks need to be centered to start motors
 * - need to be in manual mode to start the motors
 */
static inline void autopilot_arming_check_motors_on( void ) {

  switch(autopilot_arming_state) {
    case STATE_UNINIT:
      autopilot_motors_on = FALSE;
      autopilot_arming_delay_counter = 0;
      if (THROTTLE_STICK_DOWN()) {
        autopilot_arming_state = STATE_MOTORS_OFF_READY;
      }
      else {
        autopilot_arming_state = STATE_WAITING;
      }
      break;
    case STATE_WAITING:
      autopilot_motors_on = FALSE;
      autopilot_arming_delay_counter = 0;
      if (THROTTLE_STICK_DOWN()) {
        autopilot_arming_state = STATE_MOTORS_OFF_READY;
      }
      break;
    case STATE_MOTORS_OFF_READY:
      autopilot_motors_on = FALSE;
      autopilot_arming_delay_counter = 0;
      if (!THROTTLE_STICK_DOWN() &&
          rc_attitude_sticks_centered() &&
          (autopilot_mode == MODE_MANUAL || autopilot_unarmed_in_auto)) {
        autopilot_arming_state = STATE_ARMING;
      }
      break;
    case STATE_ARMING:
      autopilot_motors_on = FALSE;
      autopilot_arming_delay_counter++;
      if (THROTTLE_STICK_DOWN() ||
          !rc_attitude_sticks_centered() ||
          (autopilot_mode != MODE_MANUAL && !autopilot_unarmed_in_auto)) {
        autopilot_arming_state = STATE_MOTORS_OFF_READY;
      }
      else if (autopilot_arming_delay_counter >= AUTOPILOT_ARMING_DELAY) {
        autopilot_arming_state = STATE_MOTORS_ON;
      }
      break;
    case STATE_MOTORS_ON:
      autopilot_motors_on = TRUE;
      autopilot_arming_delay_counter = AUTOPILOT_ARMING_DELAY;
      if (THROTTLE_STICK_DOWN()) {
        autopilot_arming_state = STATE_UNARMING;
      }
      break;
    case STATE_UNARMING:
      autopilot_motors_on = TRUE;
      autopilot_arming_delay_counter--;
      if (!THROTTLE_STICK_DOWN()) {
        autopilot_arming_state = STATE_MOTORS_ON;
      }
      else if (autopilot_arming_delay_counter == 0) {
        autopilot_arming_state = STATE_MOTORS_OFF_READY;
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

#endif /* AUTOPILOT_ARMING_THROTTLE_H */
