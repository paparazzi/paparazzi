/*
 * Copyright (C) 2010 Flixr
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

#include "switching/switch_servo.h"
#include "generated/airframe.h"
#include "modules/actuators/actuators.h"

bool switch_servo_on;

// One level of macro stack to allow redefinition of the default servo
#define _SwitchServo(_n, _v) ActuatorSet(_n, _v)
#define SwitchServo(_n, _v) _SwitchServo(_n, _v)

void switch_servo_init(void)
{
  switch_servo_on = false;
  switch_servo_periodic();
}

void switch_servo_periodic(void)
{
  if (switch_servo_on == TRUE) {
    SwitchServo(SWITCH_SERVO_SERVO, SWITCH_SERVO_ON_VALUE);
  } else {
    SwitchServo(SWITCH_SERVO_SERVO, SWITCH_SERVO_OFF_VALUE);
  }
}
