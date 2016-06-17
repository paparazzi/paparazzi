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

#include "servo_switch/servo_switch.h"
#include "generated/airframe.h"
#include "subsystems/actuators.h"

bool servo_switch_on;

// One level of macro stack to allow redefinition of the default servo
#define _ServoSwitch(_n, _v) ActuatorSet(_n, _v)
#define ServoSwitch(_n, _v) _ServoSwitch(_n, _v)

void servo_switch_init(void)
{
  servo_switch_on = false;
  servo_switch_periodic();
}

void servo_switch_periodic(void)
{
  if (servo_switch_on == TRUE) {
    ServoSwitch(SERVO_SWITCH_SERVO, SERVO_SWITCH_ON_VALUE);
  } else {
    ServoSwitch(SERVO_SWITCH_SERVO, SERVO_SWITCH_OFF_VALUE);
  }
}
