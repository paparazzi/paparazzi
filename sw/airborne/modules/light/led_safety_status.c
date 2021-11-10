/*
 * Copyright (C) 2012 Pranay Sinha <psinha@transition-robotics.com>
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

/**
 * @file modules/light/led_safety_status.c
 *
 * Simple module to blink LEDs when battery voltage drops below a certain
 * level, radio control is lost or when takeoff safety conditions are not met.
 */

#include "led.h"
#include "generated/airframe.h"
#include "modules/energy/electrical.h"
#include "subsystems/radio_control.h"
#include "autopilot.h"
#include "autopilot_rc_helpers.h"

#include "modules/light/led_safety_status.h"

#ifndef SAFETY_WARNING_LED
#error You must define SAFETY_WARNING_LED to use this module!
#else

void led_safety_status_init(void)
{
  LED_ON(SAFETY_WARNING_LED);
  led_safety_status_periodic();
}

void led_safety_status_periodic(void)
{
  if (radio_control.status == RC_LOST || radio_control.status == RC_REALLY_LOST) {
    RunXTimesEvery(0, 60, 5, 7, {LED_TOGGLE(SAFETY_WARNING_LED);});
    RunXTimesEvery(130, 130, 10, 6, {LED_TOGGLE(SAFETY_WARNING_LED);});
  } else if (!(autopilot_get_mode() == MODE_MANUAL) && !autopilot_get_motors_on()) {
    RunXTimesEvery(20, 240, 40, 1, {LED_ON(SAFETY_WARNING_LED);});
    RunXTimesEvery(0, 240, 40, 1, {LED_OFF(SAFETY_WARNING_LED);});
  } else if (!THROTTLE_STICK_DOWN() && !autopilot_get_motors_on()) {
    RunXTimesEvery(20, 240, 40, 2, {LED_ON(SAFETY_WARNING_LED);});
    RunXTimesEvery(0, 240, 40, 2, {LED_OFF(SAFETY_WARNING_LED);});
  } else if (!ROLL_STICK_CENTERED() && !autopilot_get_motors_on()) {
    RunXTimesEvery(20, 240, 40, 3, {LED_ON(SAFETY_WARNING_LED);});
    RunXTimesEvery(0, 240, 40, 3, {LED_OFF(SAFETY_WARNING_LED);});
  } else if (!PITCH_STICK_CENTERED() && !autopilot_get_motors_on()) {
    RunXTimesEvery(20, 240, 40, 4, {LED_ON(SAFETY_WARNING_LED);});
    RunXTimesEvery(0, 240, 40, 4, {LED_OFF(SAFETY_WARNING_LED);});
  } else if (!YAW_STICK_CENTERED() && !autopilot_get_motors_on()) {
    RunXTimesEvery(20, 240, 40, 5, {LED_ON(SAFETY_WARNING_LED);});
    RunXTimesEvery(0, 240, 40, 5, {LED_OFF(SAFETY_WARNING_LED);});
  }
#ifdef LOW_BAT_LEVEL
  else if (electrical.vsupply < LOW_BAT_LEVEL) {
    RunOnceEvery(20, {LED_TOGGLE(SAFETY_WARNING_LED);});
  } else if (electrical.vsupply < (LOW_BAT_LEVEL + 0.5)) {
    RunXTimesEvery(0, 300, 10, 10, {LED_TOGGLE(SAFETY_WARNING_LED);});
  }
#endif
  else {
    LED_ON(SAFETY_WARNING_LED);
  }
}
#endif
