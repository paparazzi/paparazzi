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

#include "booz2_autopilot.h"

#include "booz_radio_control.h"
#include "booz2_commands.h"
#include "booz2_navigation.h"
#include "booz_guidance.h"
#include "booz_stabilization.h"

uint8_t booz2_autopilot_mode;
uint8_t booz2_autopilot_mode_auto2;
bool_t  booz2_autopilot_motors_on;
bool_t  booz2_autopilot_in_flight;
uint8_t booz2_autopilot_tol;
uint32_t booz2_autopilot_motors_on_counter;
uint32_t booz2_autopilot_in_flight_counter;

bool_t booz2_autopilot_detect_ground;

#define BOOZ2_AUTOPILOT_MOTOR_ON_TIME     40
#define BOOZ2_AUTOPILOT_IN_FLIGHT_TIME    40
#define BOOZ2_AUTOPILOT_THROTTLE_TRESHOLD (MAX_PPRZ / 20)
#define BOOZ2_AUTOPILOT_YAW_TRESHOLD      (MAX_PPRZ * 19 / 20)

void booz2_autopilot_init(void) {
  booz2_autopilot_mode = BOOZ2_AP_MODE_KILL;
  booz2_autopilot_motors_on = FALSE;
  booz2_autopilot_in_flight = FALSE;
  booz2_autopilot_motors_on_counter = 0;
  booz2_autopilot_in_flight_counter = 0;
  booz2_autopilot_mode_auto2 = BOOZ2_MODE_AUTO2;
  booz2_autopilot_tol = 0;
  booz2_autopilot_detect_ground = FALSE;
}


#if 0
#include "uart.h"
#include "downlink.h"
#include "messages.h"
#endif

void booz2_autopilot_periodic(void) {
  
  RunOnceEvery(50, nav_periodic_task_10Hz());
#ifdef BOOZ_FAILSAFE_GROUND_DETECT
  if (booz2_autopilot_mode == BOOZ2_AP_MODE_FAILSAFE && booz2_autopilot_detect_ground) {
    booz2_autopilot_set_mode(BOOZ2_AP_MODE_KILL);
    booz2_autopilot_detect_ground = FALSE;
  }
#endif
  if ( !booz2_autopilot_motors_on ||
#ifndef BOOZ_FAILSAFE_GROUND_DETECT
       booz2_autopilot_mode == BOOZ2_AP_MODE_FAILSAFE ||
#endif
       booz2_autopilot_mode == BOOZ2_AP_MODE_KILL ) {
    SetCommands(booz2_commands_failsafe, 
		booz2_autopilot_in_flight, booz2_autopilot_motors_on);
  }
  else {
    booz2_guidance_v_run( booz2_autopilot_in_flight );
    booz2_guidance_h_run( booz2_autopilot_in_flight );
    SetCommands(booz_stabilization_cmd, 
        booz2_autopilot_in_flight, booz2_autopilot_motors_on);
  }

}


void booz2_autopilot_set_mode(uint8_t new_autopilot_mode) {

  if (new_autopilot_mode != booz2_autopilot_mode) {
    /* horizontal mode */
    switch (new_autopilot_mode) {
    case BOOZ2_AP_MODE_FAILSAFE:
#ifdef BOOZ_FAILSAFE_GROUND_DETECT
      booz_stab_att_sp_euler.phi = 0;
      booz_stab_att_sp_euler.theta = 0;
      booz2_guidance_h_mode_changed(BOOZ2_GUIDANCE_H_MODE_ATTITUDE);
      break;
#endif
    case BOOZ2_AP_MODE_KILL:
      booz2_autopilot_motors_on = FALSE;
      booz2_guidance_h_mode_changed(BOOZ2_GUIDANCE_H_MODE_KILL);
      break;
    case BOOZ2_AP_MODE_RATE_DIRECT:
    case BOOZ2_AP_MODE_RATE_Z_HOLD:
      booz2_guidance_h_mode_changed(BOOZ2_GUIDANCE_H_MODE_RATE);
      break;
    case BOOZ2_AP_MODE_ATTITUDE_DIRECT:
    case BOOZ2_AP_MODE_ATTITUDE_CLIMB:
    case BOOZ2_AP_MODE_ATTITUDE_Z_HOLD:
      booz2_guidance_h_mode_changed(BOOZ2_GUIDANCE_H_MODE_ATTITUDE);
      break;
    case BOOZ2_AP_MODE_HOVER_DIRECT:
    case BOOZ2_AP_MODE_HOVER_CLIMB:
    case BOOZ2_AP_MODE_HOVER_Z_HOLD:
      booz2_guidance_h_mode_changed(BOOZ2_GUIDANCE_H_MODE_HOVER);
      break;
    case BOOZ2_AP_MODE_NAV:
      booz2_guidance_h_mode_changed(BOOZ2_GUIDANCE_H_MODE_NAV);
      break;
    }
    /* vertical mode */
    switch (new_autopilot_mode) {
    case BOOZ2_AP_MODE_FAILSAFE:
#ifdef BOOZ_FAILSAFE_GROUND_DETECT
      booz2_guidance_v_zd_sp = SPEED_BFP_OF_REAL(0.5);
      booz2_guidance_v_mode_changed(BOOZ2_GUIDANCE_V_MODE_CLIMB);
      break;
#endif
    case BOOZ2_AP_MODE_KILL:
      booz2_guidance_v_mode_changed(BOOZ2_GUIDANCE_V_MODE_KILL);
      break;
    case BOOZ2_AP_MODE_RATE_DIRECT:
    case BOOZ2_AP_MODE_ATTITUDE_DIRECT:
    case BOOZ2_AP_MODE_HOVER_DIRECT:
      booz2_guidance_v_mode_changed(BOOZ2_GUIDANCE_V_MODE_RC_DIRECT);
      break;
    case BOOZ2_AP_MODE_RATE_RC_CLIMB:
    case BOOZ2_AP_MODE_ATTITUDE_RC_CLIMB:
      booz2_guidance_v_mode_changed(BOOZ2_GUIDANCE_V_MODE_RC_CLIMB);
      break;
    case BOOZ2_AP_MODE_ATTITUDE_CLIMB:
    case BOOZ2_AP_MODE_HOVER_CLIMB:
      booz2_guidance_v_mode_changed(BOOZ2_GUIDANCE_V_MODE_CLIMB);
      break;
    case BOOZ2_AP_MODE_RATE_Z_HOLD:
    case BOOZ2_AP_MODE_ATTITUDE_Z_HOLD:
    case BOOZ2_AP_MODE_HOVER_Z_HOLD:
      booz2_guidance_v_mode_changed(BOOZ2_GUIDANCE_V_MODE_HOVER);
      break;
    case BOOZ2_AP_MODE_NAV:
      booz2_guidance_v_mode_changed(BOOZ2_GUIDANCE_V_MODE_NAV);
      break;
    }
    booz2_autopilot_mode = new_autopilot_mode;
  } 
  
}

#define THROTTLE_STICK_DOWN()						\
  (radio_control.values[RADIO_CONTROL_THROTTLE] < BOOZ2_AUTOPILOT_THROTTLE_TRESHOLD)
#define YAW_STICK_PUSHED()						\
  (radio_control.values[RADIO_CONTROL_YAW] > BOOZ2_AUTOPILOT_YAW_TRESHOLD || \
   radio_control.values[RADIO_CONTROL_YAW] < -BOOZ2_AUTOPILOT_YAW_TRESHOLD)

#define BOOZ2_AUTOPILOT_CHECK_IN_FLIGHT() {				\
    if (booz2_autopilot_in_flight) {					\
      if (booz2_autopilot_in_flight_counter > 0) {			\
	if (THROTTLE_STICK_DOWN()) {					\
	  booz2_autopilot_in_flight_counter--;				\
	  if (booz2_autopilot_in_flight_counter == 0) {			\
	    booz2_autopilot_in_flight = FALSE;				\
	  }								\
	}								\
	else {	/* !THROTTLE_STICK_DOWN */				\
	  booz2_autopilot_in_flight_counter = BOOZ2_AUTOPILOT_IN_FLIGHT_TIME; \
        }								\
      }									\
    }									\
    else { /* not in flight */						\
      if (booz2_autopilot_in_flight_counter < BOOZ2_AUTOPILOT_IN_FLIGHT_TIME && \
	  booz2_autopilot_motors_on) {					\
	if (!THROTTLE_STICK_DOWN()) {					\
	  booz2_autopilot_in_flight_counter++;				\
	  if (booz2_autopilot_in_flight_counter == BOOZ2_AUTOPILOT_IN_FLIGHT_TIME) \
	    booz2_autopilot_in_flight = TRUE;				\
	}								\
	else { /*  THROTTLE_STICK_DOWN */				\
	  booz2_autopilot_in_flight_counter = 0;			\
	}								\
      }									\
    }									\
  }

#define BOOZ2_AUTOPILOT_CHECK_MOTORS_ON() {				\
    if (booz2_autopilot_motors_on) {					\
      if (THROTTLE_STICK_DOWN() && YAW_STICK_PUSHED()) {		\
	if ( booz2_autopilot_motors_on_counter > 0) {			\
	  booz2_autopilot_motors_on_counter--;				\
	  if (booz2_autopilot_motors_on_counter == 0)			\
	    booz2_autopilot_motors_on = FALSE;				\
	}								\
      }									\
      else { /* sticks not in the corner */				\
	booz2_autopilot_motors_on_counter = BOOZ2_AUTOPILOT_MOTOR_ON_TIME; \
      }									\
    }									\
    else { /* motors off */						\
      if (THROTTLE_STICK_DOWN() && YAW_STICK_PUSHED()) {		\
	if ( booz2_autopilot_motors_on_counter <  BOOZ2_AUTOPILOT_MOTOR_ON_TIME) { \
	  booz2_autopilot_motors_on_counter++;				\
	  if (booz2_autopilot_motors_on_counter == BOOZ2_AUTOPILOT_MOTOR_ON_TIME) \
	    booz2_autopilot_motors_on = TRUE;				\
	}								\
      }									\
      else {								\
	booz2_autopilot_motors_on_counter = 0;				\
      }									\
    }									\
  }



void booz2_autopilot_on_rc_frame(void) {

#if 0
  DOWNLINK_SEND_BOOZ_DEBUG(&rc_values[RADIO_THROTTLE],		\
			   &rc_values[RADIO_ROLL],		\
			   &rc_values[RADIO_PITCH],		\
			   &rc_values[RADIO_YAW]);		
#endif
  
  uint8_t new_autopilot_mode = 0;
  BOOZ_AP_MODE_OF_PPRZ(radio_control.values[RADIO_CONTROL_MODE], new_autopilot_mode);
  booz2_autopilot_set_mode(new_autopilot_mode);
  
#ifdef RADIO_CONTROL_KILL_SWITCH
  if (radio_control.values[RADIO_CONTROL_KILL_SWITCH] < 0)
    booz2_autopilot_set_mode(BOOZ2_AP_MODE_KILL);
#endif
  
  BOOZ2_AUTOPILOT_CHECK_MOTORS_ON();
  BOOZ2_AUTOPILOT_CHECK_IN_FLIGHT();
  
  if (booz2_autopilot_mode > BOOZ2_AP_MODE_FAILSAFE) {
    booz2_guidance_v_read_rc();
    booz2_guidance_h_read_rc(booz2_autopilot_in_flight);
  }
  
}
