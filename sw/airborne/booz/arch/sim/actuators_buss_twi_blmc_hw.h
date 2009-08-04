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

#ifndef ACTUATORS_BUSS_TWI_BLMC_HW_H
#define ACTUATORS_BUSS_TWI_BLMC_HW_H

#include "airframe.h"
#include "actuators/booz_supervision.h"

#define BUSS_TWI_BLMC_NB 4
extern uint8_t buss_twi_blmc_motor_power[BUSS_TWI_BLMC_NB];

#define Actuator(i) buss_twi_blmc_motor_power[i]
#define SetActuatorsFromCommands(_motors_on) {				\
    pprz_t mixed_commands[SERVOS_NB];					\
    BOOZ2_SUPERVISION_RUN(mixed_commands, booz2_commands, _motors_on);	\
    Actuator(SERVO_FRONT) = (uint8_t)mixed_commands[SERVO_FRONT];	\
    Actuator(SERVO_BACK)  = (uint8_t)mixed_commands[SERVO_BACK];	\
    Actuator(SERVO_RIGHT) = (uint8_t)mixed_commands[SERVO_RIGHT];	\
    Actuator(SERVO_LEFT)  = (uint8_t)mixed_commands[SERVO_LEFT];	\
  }

extern uint8_t twi_blmc_nb_err;

#endif /* ACTUATORS_BUSS_TWI_BLMC_HW_H */
