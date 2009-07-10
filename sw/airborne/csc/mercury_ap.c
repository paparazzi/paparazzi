/*
 * Copyright (C) 2009 Joby Energy
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

/** \file csc_ap.c
 */

#include <inttypes.h>
#include "commands.h"
#include "mercury_xsens.h"
#include "booz2_autopilot.h"
#include "booz2_stabilization.h"
#include "booz2_stabilization_attitude.h"
#include "led.h"
#include "pprz_algebra_float.h"
#include "string.h"
#include "radio_control.h"
#include "mercury_supervision.h"
#include "actuators.h" 
#include "props_csc.h"
#include "csc_booz2_guidance_v.h"

static const int xsens_id = 0;

// based on booz2_autopilot.c - 4c4112f044adeb48c5af7afbc070863839f697c9
// -- mmt 6/15/09


uint8_t booz2_autopilot_mode;
uint8_t booz2_autopilot_mode_auto2;
bool_t  booz2_autopilot_motors_on;
bool_t  booz2_autopilot_in_flight;
uint8_t booz2_autopilot_tol;
uint32_t booz2_autopilot_motors_on_counter;
uint32_t booz2_autopilot_in_flight_counter;

uint16_t mercury_yaw_servo_gain;


#define BOOZ2_AUTOPILOT_MOTOR_ON_TIME     (512/4)
#define BOOZ2_AUTOPILOT_IN_FLIGHT_TIME    40
#define BOOZ2_AUTOPILOT_THROTTLE_TRESHOLD (MAX_PPRZ * -14 / 20)
#define BOOZ2_AUTOPILOT_YAW_TRESHOLD      (MAX_PPRZ *  14 / 20)


void csc_ap_init(void) {
  booz2_autopilot_mode = BOOZ2_AP_MODE_FAILSAFE;
  booz2_autopilot_motors_on = FALSE;
  booz2_autopilot_in_flight = FALSE;
  booz2_autopilot_motors_on_counter = 0;
  booz2_autopilot_in_flight_counter = 0;
  booz2_autopilot_mode_auto2 = BOOZ2_MODE_AUTO2;
  booz2_autopilot_tol = 0;
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
    if(!booz2_autopilot_motors_on){					\
      if (rc_values[RADIO_THROTTLE] < BOOZ2_AUTOPILOT_THROTTLE_TRESHOLD && \
	  (rc_values[RADIO_YAW] > BOOZ2_AUTOPILOT_YAW_TRESHOLD ||	\
	   rc_values[RADIO_YAW] < -BOOZ2_AUTOPILOT_YAW_TRESHOLD)) {	\
	  if ( booz2_autopilot_motors_on_counter <  BOOZ2_AUTOPILOT_MOTOR_ON_TIME) { \
	    booz2_autopilot_motors_on_counter++;			\
	    if (booz2_autopilot_motors_on_counter == BOOZ2_AUTOPILOT_MOTOR_ON_TIME){ \
	      booz2_autopilot_motors_on = TRUE;				\
	      booz2_autopilot_in_flight_counter += BOOZ2_AUTOPILOT_MOTOR_ON_TIME; \
	    }								\
	  }								\
      } else {								\
	booz2_autopilot_motors_on_counter = 0;				\
      }									\
    }									\
  }


// adapted from booz2_commands.h - cb6e74ae259a2384046f431c35735dc8018c0ecd
// mmt -- 06/16/09
const pprz_t csc_ap_commands_failsafe[COMMANDS_NB] = COMMANDS_FAILSAFE;


#define CscSetCommands(_in_cmd, _in_flight, _motors_on) {			\
    commands[COMMAND_PITCH]  = _in_cmd[COMMAND_PITCH];		\
    commands[COMMAND_ROLL]   = _in_cmd[COMMAND_ROLL];		\
    commands[COMMAND_YAW]    = (_in_flight) ? _in_cmd[COMMAND_YAW] : 0; \
    commands[COMMAND_THRUST] = (_motors_on) ? _in_cmd[COMMAND_THRUST] : 0; \
  }

pprz_t mixed_commands[PROPS_NB];


void csc_ap_periodic(uint8_t _in_flight, uint8_t kill) {
  //  RunOnceEvery(50, nav_periodic_task_10Hz())
  BOOZ2_AUTOPILOT_CHECK_MOTORS_ON();
  if(kill) booz2_autopilot_motors_on = FALSE;
  booz2_autopilot_in_flight = _in_flight;

  booz2_stabilization_attitude_read_rc(booz2_autopilot_in_flight);
  booz2_stabilization_attitude_run(booz2_autopilot_in_flight);
  booz2_guidance_v_run(booz2_autopilot_in_flight);
  
  booz2_stabilization_cmd[COMMAND_THRUST] = (int32_t)rc_values[RADIO_THROTTLE] * 105 / 7200 + 95;
 
  
  CscSetCommands(booz2_stabilization_cmd,
		 booz2_autopilot_in_flight,booz2_autopilot_motors_on);
    
    
  BOOZ2_SUPERVISION_RUN(mixed_commands, commands, booz2_autopilot_motors_on);
  props_set(PROP_UPPER_LEFT, mixed_commands[PROP_UPPER_LEFT]);
  props_set(PROP_LOWER_RIGHT,mixed_commands[PROP_LOWER_RIGHT]);
  props_set(PROP_LOWER_LEFT, mixed_commands[PROP_LOWER_LEFT]);
  props_set(PROP_UPPER_RIGHT,mixed_commands[PROP_UPPER_RIGHT]);
  props_commit();
  
  
  MERCURY_SURFACES_SUPERVISION_RUN(Actuator,
				   booz2_stabilization_cmd,
				   mixed_commands,
				   (!booz2_autopilot_in_flight));
  ActuatorsCommit();
    
}
