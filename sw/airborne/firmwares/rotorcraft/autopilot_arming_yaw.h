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
 * @file firmwares/rotorcraft/autopilot_arming_yaw.h
 *
 * Arm the motors by with max yaw stick.
 *
 */

#ifndef AUTOPILOT_ARMING_YAW_H
#define AUTOPILOT_ARMING_YAW_H

#include "autopilot_rc_helpers.h"
#include "autopilot_firmware.h"
#include "autopilot.h"

/** Delay until motors are armed/disarmed.
 * In number of rc frames received.
 * So 40 is usually ~1s.
 */
#ifndef MOTOR_ARMING_DELAY
#define MOTOR_ARMING_DELAY  40
#endif

/// Motors ON check state machine states
enum arming_state {
  STATUS_INITIALISE_RC,
  STATUS_MOTORS_AUTOMATICALLY_OFF,
  STATUS_MOTORS_AUTOMATICALLY_OFF_SAFETY_WAIT,
  STATUS_MOTORS_OFF,
  STATUS_M_OFF_STICK_PUSHED,
  STATUS_START_MOTORS,
  STATUS_MOTORS_ON,
  STATUS_M_ON_STICK_PUSHED,
  STATUS_STOP_MOTORS
};

uint32_t autopilot_motors_on_counter;
enum arming_state autopilot_check_motor_status;


static inline void autopilot_arming_init(void)
{
  autopilot_motors_on_counter = 0;
  autopilot_check_motor_status = STATUS_INITIALISE_RC;
}


/** Update the status of the check_motors state machine.
 */
static inline void autopilot_arming_set(bool motors_on)
{
  if (motors_on) {
    autopilot_check_motor_status = STATUS_MOTORS_ON;
  } else {
    autopilot_check_motor_status = STATUS_MOTORS_AUTOMATICALLY_OFF;
  }
}

#define YAW_MUST_BE_CENTERED true
#define YAW_MUST_BE_PUSHED false
/** Checks all arm requirements and returns true if OK and false otherwise.
 *  Also sets the arming status to provide information to the user
 *
 *  @param[in] yaw_must_be_centered check is vallid of a yaw stick centered (true) or pushed (false)
 *  @return true if arming checks are all valid
 */
static inline bool autopilot_arming_check_valid(bool yaw_must_be_centered)
{
  if (!THROTTLE_STICK_DOWN()) {
    autopilot.arming_status = AP_ARMING_STATUS_THROTTLE_NOT_DOWN;
  } else if (!PITCH_STICK_CENTERED()) {
    autopilot.arming_status = AP_ARMING_STATUS_PITCH_NOT_CENTERED;
  } else if (!ROLL_STICK_CENTERED()) {
    autopilot.arming_status = AP_ARMING_STATUS_ROLL_NOT_CENTERED;
  } else {
    if (yaw_must_be_centered && !YAW_STICK_CENTERED()) {
      autopilot.arming_status = AP_ARMING_STATUS_YAW_NOT_CENTERED;
    } else if (!yaw_must_be_centered && YAW_STICK_CENTERED()) {
      autopilot.arming_status = AP_ARMING_STATUS_YAW_CENTERED;
    } else {
      return true; // all checks valid
    }
  }
  return false; // one of the checks failed
}

/**
 * State machine to check if motors should be turned ON or OFF.
 * The motors start/stop when pushing the yaw stick without throttle until #MOTOR_ARMING_DELAY is reached.
 * An intermediate state prevents oscillating between ON and OFF while keeping the stick pushed.
 * The stick must return to a neutral position before starting/stoping again.
 */
static inline void autopilot_arming_check_motors_on(void)
{
  /* only allow switching motor if not in KILL mode */
  if (autopilot_get_mode() != AP_MODE_KILL) {

    switch (autopilot_check_motor_status) {
      case STATUS_INITIALISE_RC: // Wait until RC is initialised (it being centered is a good pointer to this)
        if (autopilot_arming_check_valid(YAW_MUST_BE_CENTERED)) {
          autopilot_check_motor_status = STATUS_MOTORS_OFF;
        }
        break;
      case STATUS_MOTORS_AUTOMATICALLY_OFF: // Motors were disarmed externally
        //(possibly due to crash)
        //wait extra delay before enabling the normal arming state machine
        autopilot.motors_on = false;
        autopilot_motors_on_counter = 0;
        if (autopilot_arming_check_valid(YAW_MUST_BE_CENTERED)) {
          autopilot_check_motor_status = STATUS_MOTORS_AUTOMATICALLY_OFF_SAFETY_WAIT;
        }
        break;
      case STATUS_MOTORS_AUTOMATICALLY_OFF_SAFETY_WAIT:
        autopilot_motors_on_counter++;
        if (autopilot_motors_on_counter >= MOTOR_ARMING_DELAY) {
          autopilot_check_motor_status = STATUS_MOTORS_OFF;
        } else {
          autopilot.arming_status = AP_ARMING_STATUS_WAITING;
        }
        break;
      case STATUS_MOTORS_OFF:
        autopilot.motors_on = false;
        autopilot_motors_on_counter = 0;
        autopilot.arming_status = AP_ARMING_STATUS_WAITING;
        if (autopilot_arming_check_valid(YAW_MUST_BE_PUSHED)) { // stick pushed
          autopilot_check_motor_status = STATUS_M_OFF_STICK_PUSHED;
        }
        break;
      case STATUS_M_OFF_STICK_PUSHED:
        autopilot.motors_on = false;
        autopilot_motors_on_counter++;
        if (autopilot_motors_on_counter >= MOTOR_ARMING_DELAY) {
          autopilot_check_motor_status = STATUS_START_MOTORS;
        } else if (!autopilot_arming_check_valid(YAW_MUST_BE_PUSHED)) { // stick released too soon
          autopilot_check_motor_status = STATUS_MOTORS_OFF;
        } else {
          autopilot.arming_status = AP_ARMING_STATUS_WAITING;
        }
        break;
      case STATUS_START_MOTORS:
        autopilot.motors_on = true;
        autopilot_motors_on_counter = MOTOR_ARMING_DELAY;
        autopilot_set_in_flight(false);   // stop fc from starting control (integration and yaw) till arm process is complete
        if (YAW_STICK_CENTERED()) { // wait until stick released
          autopilot_check_motor_status = STATUS_MOTORS_ON;
        }
        break;
      case STATUS_MOTORS_ON:
        autopilot.arming_status = AP_ARMING_STATUS_ARMED;
        autopilot.motors_on = true;
        autopilot_motors_on_counter = MOTOR_ARMING_DELAY;
        if (THROTTLE_STICK_DOWN() && YAW_STICK_PUSHED()) { // stick pushed
          autopilot_check_motor_status = STATUS_M_ON_STICK_PUSHED;
        }
        break;
      case STATUS_M_ON_STICK_PUSHED:
        autopilot.motors_on = true;
        autopilot_motors_on_counter--;
        if (autopilot_motors_on_counter == 0) {
          autopilot_check_motor_status = STATUS_STOP_MOTORS;
        } else if (!(THROTTLE_STICK_DOWN() && YAW_STICK_PUSHED())) { // stick released too soon
          autopilot_check_motor_status = STATUS_MOTORS_ON;
        } else {
          autopilot.arming_status = AP_ARMING_STATUS_DISARMING;
        }
        break;
      case STATUS_STOP_MOTORS:
        autopilot.motors_on = false;
        autopilot_motors_on_counter = 0;
        if (autopilot_arming_check_valid(YAW_MUST_BE_CENTERED)) { // wait till release disarm stick before allowing to re-arm
          autopilot_check_motor_status = STATUS_MOTORS_OFF;
        }
        break;
      default:
        break;
    }
  } else {
    autopilot.arming_status = AP_ARMING_STATUS_KILLED;
  }
}

#endif /* AUTOPILOT_ARMING_YAW_H */
