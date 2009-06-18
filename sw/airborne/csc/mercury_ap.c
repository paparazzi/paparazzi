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
#include "led.h"
#include "pprz_algebra_float.h"
#include "string.h"
#include "radio_control.h"
#include "mercury_supervision.h"
#include "actuators.h"

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


#define BOOZ2_AUTOPILOT_MOTOR_ON_TIME     40
#define BOOZ2_AUTOPILOT_IN_FLIGHT_TIME    40
#define BOOZ2_AUTOPILOT_THROTTLE_TRESHOLD (MAX_PPRZ / 20)
#define BOOZ2_AUTOPILOT_YAW_TRESHOLD      (MAX_PPRZ * 19 / 20)


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


// adapted from booz2_commands.h - cb6e74ae259a2384046f431c35735dc8018c0ecd
// mmt -- 06/16/09
pprz_t csc_ap_commands[COMMANDS_NB];
const pprz_t csc_ap_commands_failsafe[COMMANDS_NB] = COMMANDS_FAILSAFE;


#define CscSetCommands(_in_cmd, _in_flight, _motors_on) {			\
    csc_ap_commands[COMMAND_PITCH]  = _in_cmd[COMMAND_PITCH];		\
    csc_ap_commands[COMMAND_ROLL]   = _in_cmd[COMMAND_ROLL];		\
    csc_ap_commands[COMMAND_YAW]    = (_in_flight) ? _in_cmd[COMMAND_YAW] : 0; \
    csc_ap_commands[COMMAND_THRUST] = (_motors_on) ? _in_cmd[COMMAND_THRUST] : 0; \
  }

pprz_t mixed_commands[SERVOS_NB];


void csc_ap_periodic(void) {
  //  RunOnceEvery(50, nav_periodic_task_10Hz())
  booz2_autopilot_motors_on = 1; // HACK for now mmt
  
  booz2_stabilization_attitude_run(booz2_autopilot_in_flight);
  
  /*   if ( !booz2_autopilot_motors_on || */
/*   if(  booz2_autopilot_mode == BOOZ2_AP_MODE_FAILSAFE || */
/*        booz2_autopilot_mode == BOOZ2_AP_MODE_KILL ) { */
/*     CscSetCommands(csc_ap_commands_failsafe, */
/* 		   booz2_autopilot_in_flight, booz2_autopilot_motors_on); */
    
/*   } else { */
    booz2_stabilization_attitude_read_rc(booz2_autopilot_in_flight);
    booz2_stabilization_cmd[COMMAND_THRUST] = (int32_t)rc_values[RADIO_THROTTLE] * 200 / MAX_PPRZ;
    
    CscSetCommands(booz2_stabilization_cmd,
		   booz2_autopilot_in_flight, booz2_autopilot_motors_on);
    //  }
  
  // TODO insert calls to servos_csc
  // mix motors
    
    BOOZ2_SUPERVISION_RUN(mixed_commands, csc_ap_commands, booz2_autopilot_motors_on); 
    Actuator(SERVO_UPPER_LEFT) = (uint8_t)mixed_commands[SERVO_UPPER_LEFT];	
    Actuator(SERVO_LOWER_RIGHT)= (uint8_t)mixed_commands[SERVO_LOWER_RIGHT];	
    Actuator(SERVO_LOWER_LEFT) = (uint8_t)mixed_commands[SERVO_LOWER_LEFT];	
    Actuator(SERVO_UPPER_RIGHT)= (uint8_t)mixed_commands[SERVO_UPPER_RIGHT];	
    ActuatorsCommit();							
}
