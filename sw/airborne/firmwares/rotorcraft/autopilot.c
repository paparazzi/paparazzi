/*
 * $Id$
 *
 * Copyright (C) 2008-2009 Antoine Drouin <poinix@gmail.com>
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

#include "firmwares/rotorcraft/autopilot.h"

#include "subsystems/radio_control.h"
#include "firmwares/rotorcraft/commands.h"
#include "firmwares/rotorcraft/navigation.h"
#include "firmwares/rotorcraft/guidance.h"
#include "firmwares/rotorcraft/stabilization.h"
#include "led.h"

uint8_t  autopilot_mode;
uint8_t  autopilot_mode_auto2;
bool_t   autopilot_motors_on;
bool_t   autopilot_in_flight;
uint32_t autopilot_motors_on_counter;
uint32_t autopilot_in_flight_counter;
uint8_t  autopilot_check_motor_status;
bool_t   kill_throttle;
bool_t   autopilot_rc;

bool_t   autopilot_power_switch;

bool_t   autopilot_detect_ground;
bool_t   autopilot_detect_ground_once;

uint16_t autopilot_flight_time;

#define AUTOPILOT_MOTOR_ON_TIME     40
#define AUTOPILOT_IN_FLIGHT_TIME    40
#define AUTOPILOT_THROTTLE_TRESHOLD (MAX_PPRZ / 20)
#define AUTOPILOT_YAW_TRESHOLD      (MAX_PPRZ * 19 / 20)
// Motors ON check state machine
#define STATUS_MOTORS_OFF           0
#define STATUS_M_OFF_STICK_PUSHED   1
#define STATUS_START_MOTORS         2
#define STATUS_MOTORS_ON            3
#define STATUS_M_ON_STICK_PUSHED    4
#define STATUS_STOP_MOTORS          5

void autopilot_init(void) {
  autopilot_mode = AP_MODE_KILL;
  autopilot_motors_on = FALSE;
  autopilot_in_flight = FALSE;
  kill_throttle = ! autopilot_motors_on;
  autopilot_motors_on_counter = 0;
  autopilot_in_flight_counter = 0;
  autopilot_check_motor_status = STATUS_MOTORS_OFF;
  autopilot_mode_auto2 = MODE_AUTO2;
  autopilot_detect_ground = FALSE;
  autopilot_detect_ground_once = FALSE;
  autopilot_flight_time = 0;
  autopilot_rc = TRUE;
  autopilot_power_switch = FALSE;
#ifdef POWER_SWITCH_LED
  LED_ON(POWER_SWITCH_LED); // POWER OFF
#endif
}


void autopilot_periodic(void) {

  RunOnceEvery(NAV_PRESCALER, nav_periodic_task());
#ifdef FAILSAFE_GROUND_DETECT
  if (autopilot_mode == AP_MODE_FAILSAFE && autopilot_detect_ground) {
    autopilot_set_mode(AP_MODE_KILL);
    autopilot_detect_ground = FALSE;
  }
#endif
  if ( !autopilot_motors_on ||
#ifndef FAILSAFE_GROUND_DETECT
       autopilot_mode == AP_MODE_FAILSAFE ||
#endif
       autopilot_mode == AP_MODE_KILL ) {
    SetCommands(commands_failsafe,
		autopilot_in_flight, autopilot_motors_on);
  }
  else {
    guidance_v_run( autopilot_in_flight );
    guidance_h_run( autopilot_in_flight );
    SetCommands(stabilization_cmd,
        autopilot_in_flight, autopilot_motors_on);
  }

}


void autopilot_set_mode(uint8_t new_autopilot_mode) {

  if (new_autopilot_mode != autopilot_mode) {
    /* horizontal mode */
    switch (new_autopilot_mode) {
    case AP_MODE_FAILSAFE:
#ifndef KILL_AS_FAILSAFE
      stab_att_sp_euler.phi = 0;
      stab_att_sp_euler.theta = 0;
      guidance_h_mode_changed(GUIDANCE_H_MODE_ATTITUDE);
      break;
#endif
    case AP_MODE_KILL:
      autopilot_motors_on = FALSE;
      guidance_h_mode_changed(GUIDANCE_H_MODE_KILL);
      break;
    case AP_MODE_RATE_DIRECT:
    case AP_MODE_RATE_Z_HOLD:
      guidance_h_mode_changed(GUIDANCE_H_MODE_RATE);
      break;
    case AP_MODE_ATTITUDE_DIRECT:
    case AP_MODE_ATTITUDE_CLIMB:
    case AP_MODE_ATTITUDE_Z_HOLD:
      guidance_h_mode_changed(GUIDANCE_H_MODE_ATTITUDE);
      break;
    case AP_MODE_HOVER_DIRECT:
    case AP_MODE_HOVER_CLIMB:
    case AP_MODE_HOVER_Z_HOLD:
      guidance_h_mode_changed(GUIDANCE_H_MODE_HOVER);
      break;
    case AP_MODE_NAV:
      guidance_h_mode_changed(GUIDANCE_H_MODE_NAV);
      break;
    default:
      break;
    }
    /* vertical mode */
    switch (new_autopilot_mode) {
    case AP_MODE_FAILSAFE:
#ifndef KILL_AS_FAILSAFE
      guidance_v_zd_sp = SPEED_BFP_OF_REAL(0.5);
      guidance_v_mode_changed(GUIDANCE_V_MODE_CLIMB);
      break;
#endif
    case AP_MODE_KILL:
      guidance_v_mode_changed(GUIDANCE_V_MODE_KILL);
      break;
    case AP_MODE_RATE_DIRECT:
    case AP_MODE_ATTITUDE_DIRECT:
    case AP_MODE_HOVER_DIRECT:
      guidance_v_mode_changed(GUIDANCE_V_MODE_RC_DIRECT);
      break;
    case AP_MODE_RATE_RC_CLIMB:
    case AP_MODE_ATTITUDE_RC_CLIMB:
      guidance_v_mode_changed(GUIDANCE_V_MODE_RC_CLIMB);
      break;
    case AP_MODE_ATTITUDE_CLIMB:
    case AP_MODE_HOVER_CLIMB:
      guidance_v_mode_changed(GUIDANCE_V_MODE_CLIMB);
      break;
    case AP_MODE_RATE_Z_HOLD:
    case AP_MODE_ATTITUDE_Z_HOLD:
    case AP_MODE_HOVER_Z_HOLD:
      guidance_v_mode_changed(GUIDANCE_V_MODE_HOVER);
      break;
    case AP_MODE_NAV:
      guidance_v_mode_changed(GUIDANCE_V_MODE_NAV);
      break;
    default:
      break;
    }
    autopilot_mode = new_autopilot_mode;
  }

}

#define THROTTLE_STICK_DOWN()						\
  (radio_control.values[RADIO_THROTTLE] < AUTOPILOT_THROTTLE_TRESHOLD)
#define YAW_STICK_PUSHED()						\
  (radio_control.values[RADIO_YAW] > AUTOPILOT_YAW_TRESHOLD || \
   radio_control.values[RADIO_YAW] < -AUTOPILOT_YAW_TRESHOLD)


static inline void autopilot_check_in_flight( void) {
  if (autopilot_in_flight) {
    if (autopilot_in_flight_counter > 0) {
      if (THROTTLE_STICK_DOWN()) {
        autopilot_in_flight_counter--;
        if (autopilot_in_flight_counter == 0) {
          autopilot_in_flight = FALSE;
        }
      }
      else {	/* !THROTTLE_STICK_DOWN */
        autopilot_in_flight_counter = AUTOPILOT_IN_FLIGHT_TIME;
      }
    }
  }
  else { /* not in flight */
    if (autopilot_in_flight_counter < AUTOPILOT_IN_FLIGHT_TIME &&
        autopilot_motors_on) {
      if (!THROTTLE_STICK_DOWN()) {
        autopilot_in_flight_counter++;
        if (autopilot_in_flight_counter == AUTOPILOT_IN_FLIGHT_TIME)
          autopilot_in_flight = TRUE;
      }
      else { /*  THROTTLE_STICK_DOWN */
        autopilot_in_flight_counter = 0;
      }
    }
  }
}

#ifdef AUTOPILOT_KILL_WITHOUT_AHRS
#include "subsystems/ahrs.h"
static inline int ahrs_is_aligned(void) {
  return (ahrs.status == AHRS_RUNNING);
}
#else
static inline int ahrs_is_aligned(void) {
  return TRUE;
}
#endif

/** Set motors ON or OFF and change the status of the check_motors state machine
 */
void autopilot_set_motors_on(bool_t motors_on) {
  autopilot_motors_on = motors_on;
  kill_throttle = ! autopilot_motors_on;
  if (autopilot_motors_on) autopilot_check_motor_status = STATUS_MOTORS_ON;
  else autopilot_check_motor_status = STATUS_MOTORS_OFF;
}

/**
 * State machine to check if motors should be turned ON or OFF
 * The motors start/stop when pushing the yaw stick without throttle during a given time
 * An intermediate state prevents oscillating between ON and OFF while keeping the stick pushed
 * The stick must return to a neutral position before starting/stoping again
 */
static inline void autopilot_check_motors_on( void ) {
  switch(autopilot_check_motor_status) {
    case STATUS_MOTORS_OFF:
      autopilot_motors_on = FALSE;
      autopilot_motors_on_counter = 0;
      if (THROTTLE_STICK_DOWN() && YAW_STICK_PUSHED()) // stick pushed
        autopilot_check_motor_status = STATUS_M_OFF_STICK_PUSHED;
      break;
    case STATUS_M_OFF_STICK_PUSHED:
      autopilot_motors_on = FALSE;
      autopilot_motors_on_counter++;
      if (autopilot_motors_on_counter >= AUTOPILOT_MOTOR_ON_TIME)
        autopilot_check_motor_status = STATUS_START_MOTORS;
      else if (!(THROTTLE_STICK_DOWN() && YAW_STICK_PUSHED())) // stick released too soon
        autopilot_check_motor_status = STATUS_MOTORS_OFF;
      break;
    case STATUS_START_MOTORS:
      autopilot_motors_on = TRUE;
      autopilot_motors_on_counter = AUTOPILOT_MOTOR_ON_TIME;
      if (!(THROTTLE_STICK_DOWN() && YAW_STICK_PUSHED())) // wait until stick released
        autopilot_check_motor_status = STATUS_MOTORS_ON;
      break;
    case STATUS_MOTORS_ON:
      autopilot_motors_on = TRUE;
      autopilot_motors_on_counter = AUTOPILOT_MOTOR_ON_TIME;
      if (THROTTLE_STICK_DOWN() && YAW_STICK_PUSHED()) // stick pushed
        autopilot_check_motor_status = STATUS_M_ON_STICK_PUSHED;
      break;
    case STATUS_M_ON_STICK_PUSHED:
      autopilot_motors_on = TRUE;
      autopilot_motors_on_counter--;
      if (autopilot_motors_on_counter == 0)
        autopilot_check_motor_status = STATUS_STOP_MOTORS;
      else if (!(THROTTLE_STICK_DOWN() && YAW_STICK_PUSHED())) // stick released too soon
        autopilot_check_motor_status = STATUS_MOTORS_ON;
      break;
    case STATUS_STOP_MOTORS:
      autopilot_motors_on = FALSE;
      autopilot_motors_on_counter = 0;
      if (!(THROTTLE_STICK_DOWN() && YAW_STICK_PUSHED())) // wait until stick released
        autopilot_check_motor_status = STATUS_MOTORS_OFF;
      break;
  };
}


void autopilot_on_rc_frame(void) {

  uint8_t new_autopilot_mode = 0;
  AP_MODE_OF_PPRZ(radio_control.values[RADIO_MODE], new_autopilot_mode);
  autopilot_set_mode(new_autopilot_mode);

#ifdef RADIO_KILL_SWITCH
  if (radio_control.values[RADIO_KILL_SWITCH] < 0)
    autopilot_set_mode(AP_MODE_KILL);
#endif

#ifdef AUTOPILOT_KILL_WITHOUT_AHRS
  if (!ahrs_is_aligned())
    autopilot_set_mode(AP_MODE_KILL);
#endif

  autopilot_check_motors_on();
  autopilot_check_in_flight();
  kill_throttle = !autopilot_motors_on;

  if (autopilot_mode > AP_MODE_FAILSAFE) {
    guidance_v_read_rc();
    guidance_h_read_rc(autopilot_in_flight);
  }

}
