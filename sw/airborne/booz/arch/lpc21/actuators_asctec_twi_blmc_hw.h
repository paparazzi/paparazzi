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
 */

#ifndef ACTUATORS_ASCTEC_TWI_BLMC_H
#define ACTUATORS_ASCTEC_TWI_BLMC_H

#include "std.h"
#include "led.h"

#include "airframe.h"

extern void asctec_twi_controller_send(void);

extern uint8_t twi_blmc_nb_err;
extern int8_t asctec_twi_blmc_motor_power[];

#define MB_TWI_CONTROLLER_COMMAND_NONE     0
#define MB_TWI_CONTROLLER_COMMAND_TEST     1
#define MB_TWI_CONTROLLER_COMMAND_REVERSE  2
#define MB_TWI_CONTROLLER_COMMAND_SET_ADDR 3


extern uint8_t actuators_asctec_twi_blmc_command;
extern uint8_t actuators_asctec_twi_blmc_addr;
extern uint8_t actuators_asctec_twi_blmc_new_addr;

#define actuators_asctec_twi_blmc_hw_SetCommand(value) { \
    actuators_asctec_twi_blmc_command = value;		 \
  }

#define actuators_asctec_twi_blmc_hw_SetAddr(value) {			\
    actuators_asctec_twi_blmc_command = MB_TWI_CONTROLLER_COMMAND_SET_ADDR; \
    actuators_asctec_twi_blmc_new_addr = value;				\
  }

#ifndef SetActuatorsFromCommands
#ifdef KILL_MOTORS
#define SetActuatorsFromCommands(_motors_on) {		      \
    Actuator(SERVO_PITCH)  = 0;				      \
    Actuator(SERVO_ROLL)   = 0;				      \
    Actuator(SERVO_YAW)    = 0;				      \
    Actuator(SERVO_THRUST) = 0;				      \
    ActuatorsCommit();					      \
  }
#else /* ! KILL_MOTORS */
#ifndef SUPERVISION_HACK_45
#define SetActuatorsFromCommands(_motors_on) {				\
    booz2_commands[COMMAND_PITCH] += SUPERVISION_TRIM_E;		\
    booz2_commands[COMMAND_ROLL]  += SUPERVISION_TRIM_A;		\
    booz2_commands[COMMAND_YAW]   += SUPERVISION_TRIM_R;		\
    Bound(booz2_commands[COMMAND_PITCH],-100, 100);			\
    Bound(booz2_commands[COMMAND_ROLL], -100, 100);			\
    Bound(booz2_commands[COMMAND_YAW],  -100, 100);			\
    if (_motors_on) {							\
      Bound(booz2_commands[COMMAND_THRUST],  1, 200);			\
    }									\
    Actuator(SERVO_PITCH)  = -(uint8_t)booz2_commands[COMMAND_PITCH];	\
    Actuator(SERVO_ROLL)   =  (uint8_t)booz2_commands[COMMAND_ROLL];	\
    Actuator(SERVO_YAW)    = -(uint8_t)booz2_commands[COMMAND_YAW];	\
    Actuator(SERVO_THRUST) =  (uint8_t)booz2_commands[COMMAND_THRUST];	\
    ActuatorsCommit();							\
  }
#else /* SUPERVISION_HACK_45 */
#define SetActuatorsFromCommands(_motors_on) {				\
    int32_t _r1 =  booz2_commands[COMMAND_ROLL] +  booz2_commands[COMMAND_PITCH]; \
    int32_t _p1 =  booz2_commands[COMMAND_ROLL] -  booz2_commands[COMMAND_PITCH]; \
    int32_t _y1 =  booz2_commands[COMMAND_YAW];				\
    int32_t _t1 =  booz2_commands[COMMAND_THRUST];			\
    Bound(_r1,-100, 100);						\
    Bound(_p1,-100, 100);						\
    Bound(_y1,-100, 100);						\
    if (_motors_on) {							\
      Bound(_t1,  1, 200);						\
    }									\
    Actuator(SERVO_ROLL)   = _r1;					\
    Actuator(SERVO_PITCH)  = _p1;					\
    Actuator(SERVO_YAW)    = _y1;					\
    Actuator(SERVO_THRUST) = _t1;					\
    ActuatorsCommit();							\
  }
#endif /* SUPERVISION_HACK_45 */
#endif /* KILL_MOTORS              */
#endif /* SetActuatorsFromCommands */

#define Actuator(i) asctec_twi_blmc_motor_power[i]
#define ActuatorsCommit() {			                \
    asctec_twi_controller_send();				\
  }

#endif /* ACTUATORS_ASCTEC_TWI_BLMC_H */
