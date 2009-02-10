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

#include "booz2_autopilot.h"

#include "radio_control.h"
#include "booz2_commands.h"
#include "booz2_navigation.h"
#include "booz2_guidance_h.h"
#include "booz2_guidance_v.h"
#include "booz2_stabilization.h"

uint8_t booz2_autopilot_mode;
uint8_t booz2_autopilot_mode_auto2;
bool_t  booz2_autopilot_motors_on;
bool_t  booz2_autopilot_in_flight;
uint32_t booz2_autopilot_motors_on_counter;
uint32_t booz2_autopilot_in_flight_counter;

#define BOOZ2_AUTOPILOT_MOTOR_ON_TIME     40
#define BOOZ2_AUTOPILOT_IN_FLIGHT_TIME    40
#define BOOZ2_AUTOPILOT_THROTTLE_TRESHOLD (MAX_PPRZ / 20)
#define BOOZ2_AUTOPILOT_YAW_TRESHOLD      (MAX_PPRZ * 19 / 20)

void booz2_autopilot_init(void) {
  booz2_autopilot_mode = BOOZ2_AP_MODE_FAILSAFE;
  booz2_autopilot_motors_on = FALSE;
  booz2_autopilot_in_flight = FALSE;
  booz2_autopilot_motors_on_counter = 0;
  booz2_autopilot_in_flight_counter = 0;
  booz2_autopilot_mode_auto2 = BOOZ2_MODE_AUTO2;
}


#if 0
#include "uart.h"
#include "downlink.h"
#include "messages.h"
#endif

void booz2_autopilot_periodic(void) {
  
  if ( !booz2_autopilot_motors_on ||
       booz2_autopilot_mode == BOOZ2_AP_MODE_FAILSAFE ||
       booz2_autopilot_mode == BOOZ2_AP_MODE_KILL ) {
    SetCommands(booz2_commands_failsafe, 
		booz2_autopilot_in_flight, booz2_autopilot_motors_on);
  }
  else {
    RunOnceEvery(50, nav_periodic_task_10Hz())
    booz2_guidance_v_run( booz2_autopilot_in_flight );
    booz2_guidance_h_run( booz2_autopilot_in_flight );
    SetCommands(booz2_stabilization_cmd, 
		booz2_autopilot_in_flight, booz2_autopilot_motors_on);
  }

}


void booz2_autopilot_set_mode(uint8_t new_autopilot_mode) {

  if (new_autopilot_mode != booz2_autopilot_mode) {
    /* horizontal mode */
    switch (new_autopilot_mode) {
    case BOOZ2_AP_MODE_FAILSAFE:
    case BOOZ2_AP_MODE_KILL:
      booz2_autopilot_motors_on = FALSE;
      booz2_guidance_h_mode_changed(BOOZ2_GUIDANCE_H_MODE_KILL);
      break;
    case BOOZ2_AP_MODE_RATE_DIRECT:
    case BOOZ2_AP_MODE_RATE_Z_HOLD:
      booz2_guidance_h_mode_changed(BOOZ2_GUIDANCE_H_MODE_RATE);
      break;
    case BOOZ2_AP_MODE_ATTITUDE_DIRECT:
    case BOOZ2_AP_MODE_ATTITUDE_Z_HOLD:
      booz2_guidance_h_mode_changed(BOOZ2_GUIDANCE_H_MODE_ATTITUDE);
      break;
    case BOOZ2_AP_MODE_HOVER_DIRECT:
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
    case BOOZ2_AP_MODE_KILL:
      booz2_guidance_v_mode_changed(BOOZ2_GUIDANCE_V_MODE_KILL);
      break;
    case BOOZ2_AP_MODE_RATE_DIRECT:
    case BOOZ2_AP_MODE_ATTITUDE_DIRECT:
    case BOOZ2_AP_MODE_HOVER_DIRECT:
      booz2_guidance_v_mode_changed(BOOZ2_GUIDANCE_V_MODE_DIRECT);
      break;
    case BOOZ2_AP_MODE_RATE_Z_HOLD:
    case BOOZ2_AP_MODE_ATTITUDE_Z_HOLD:
    case BOOZ2_AP_MODE_HOVER_Z_HOLD:
    case BOOZ2_AP_MODE_NAV:
      booz2_guidance_v_mode_changed(BOOZ2_GUIDANCE_V_MODE_HOVER);
      break;
    }
    booz2_autopilot_mode = new_autopilot_mode;
  } 
  
}

#define BOOZ2_AUTOPILOT_CHECK_IN_FLIGHT() {				\
    if (booz2_autopilot_in_flight) {					\
      if (booz2_autopilot_in_flight_counter > 0) {			\
	if (rc_values[RADIO_THROTTLE] < BOOZ2_AUTOPILOT_THROTTLE_TRESHOLD) { \
	  booz2_autopilot_in_flight_counter--;				\
	  if (booz2_autopilot_in_flight_counter == 0) {			\
	    booz2_autopilot_in_flight = FALSE;				\
	  }								\
	}								\
	else {	/* rc throttle > threshold */				\
	  booz2_autopilot_in_flight_counter = BOOZ2_AUTOPILOT_IN_FLIGHT_TIME; \
        }								\
      }									\
    }									\
    else { /* not in flight */						\
      if (booz2_autopilot_in_flight_counter < BOOZ2_AUTOPILOT_IN_FLIGHT_TIME && \
	  booz2_autopilot_motors_on) {					\
	if (rc_values[RADIO_THROTTLE] > BOOZ2_AUTOPILOT_THROTTLE_TRESHOLD) { \
	  booz2_autopilot_in_flight_counter++;				\
	  if (booz2_autopilot_in_flight_counter == BOOZ2_AUTOPILOT_IN_FLIGHT_TIME) \
	    booz2_autopilot_in_flight = TRUE;				\
	}								\
	else { /*  rc throttle < threshold */				\
	  booz2_autopilot_in_flight_counter = 0;			\
	}								\
      }									\
    }									\
  }

#define BOOZ2_AUTOPILOT_CHECK_MOTORS_ON() {				\
    if (booz2_autopilot_motors_on) {					\
      if (rc_values[RADIO_THROTTLE] < BOOZ2_AUTOPILOT_THROTTLE_TRESHOLD && \
	  (rc_values[RADIO_YAW] > BOOZ2_AUTOPILOT_YAW_TRESHOLD ||	\
	   rc_values[RADIO_YAW] < -BOOZ2_AUTOPILOT_YAW_TRESHOLD)) {	\
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
      if (rc_values[RADIO_THROTTLE] < BOOZ2_AUTOPILOT_THROTTLE_TRESHOLD && \
	  (rc_values[RADIO_YAW] > BOOZ2_AUTOPILOT_YAW_TRESHOLD ||	\
	   rc_values[RADIO_YAW] < -BOOZ2_AUTOPILOT_YAW_TRESHOLD)) {	\
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



void booz2_autopilot_on_rc_event(void) {

#if 0
      DOWNLINK_SEND_BOOZ_DEBUG(&rc_values[RADIO_THROTTLE],	\
			       &rc_values[RADIO_ROLL],		\
			       &rc_values[RADIO_PITCH],		\
			       &rc_values[RADIO_YAW]);		
#endif

  /* I think this should be hidden in rc code */
  /* the ap gets a mode everytime - the rc filters it */
  if (rc_values_contains_avg_channels) {
    uint8_t new_autopilot_mode = 0;
    BOOZ_AP_MODE_OF_PPRZ(rc_values[RADIO_MODE],new_autopilot_mode);
    booz2_autopilot_set_mode(new_autopilot_mode);
    rc_values_contains_avg_channels = FALSE;
  }

#ifdef KILL_SWITCH
  if (rc_values[KILL_SWITCH] < 0)
    booz2_autopilot_set_mode(BOOZ2_AP_MODE_KILL);
#endif

  BOOZ2_AUTOPILOT_CHECK_MOTORS_ON();
  BOOZ2_AUTOPILOT_CHECK_IN_FLIGHT();

  booz2_guidance_v_read_rc();
  booz2_guidance_h_read_rc(booz2_autopilot_in_flight);

}
