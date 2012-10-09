/*
 * Copyright (C) 2010 The Paparazzi Team
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

#include "generated/airframe.h"

#include "firmwares/rotorcraft/actuators.h"
#include "firmwares/rotorcraft/commands.h"
#include "subsystems/actuators/actuators_pwm.h"


#define ESC_STOPPED SERVOS_TICS_OF_USEC(0)

#ifndef KILL_MOTORS
#define ESC_HOVER   SERVOS_TICS_OF_USEC(5500)
#else
#define ESC_HOVER   SERVOS_TICS_OF_USEC(0)
#endif

int32_t actuators_pwm_values[ACTUATORS_PWM_NB];

void actuators_init(void)
{
  actuators_pwm_arch_init();
}


void actuators_set(bool_t motors_on)
{
  SetActuatorsFromCommands(commands);
}

