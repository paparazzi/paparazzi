/*
 * $Id$
 *
 * Copyright (C) 2012  Thomas Kolb
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

#include "bat_checker.h"
#include "generated/airframe.h"
#include "subsystems/electrical.h"
#include "led.h"

#ifndef CRITIC_BAT_LEVEL
#error You must define CRITIC_BAT_LEVEL to use this module!
#endif

#ifndef LOW_BAT_LEVEL
#error You must define LOW_BAT_LEVEL to use this module!
#endif

#ifndef BAT_CHECKER_LED
#error You must define BAT_CHECKER_LED in your airframe file.
#endif

#ifndef BAT_CHECKER_DELAY
#warning BAT_CHECKER_DELAY is undefined. Falling back to 5 seconds.
#define BAT_CHECKER_DELAY 5
#endif

// at this level, the buzzer will be activated periodically
#define WARN_BAT_LEVEL1 (LOW_BAT_LEVEL*10)

// at this level, the buzzer will be activated permanently
#define WARN_BAT_LEVEL2 (CRITIC_BAT_LEVEL*10)

#pragma message "Battery checker included!"

void init_bat_checker(void) {
  LED_INIT(BAT_CHECKER_LED);
  LED_OFF(BAT_CHECKER_LED);
}

void bat_checker_periodic(void) {
  static uint8_t bat_low_counter = 0;
  if(electrical.vsupply < WARN_BAT_LEVEL1) {
    if(bat_low_counter)
      bat_low_counter--;
  } else {
    bat_low_counter = BAT_CHECKER_DELAY * bat_checker_periodic_FREQ;
  }

  if(!bat_low_counter) {
    if(electrical.vsupply < WARN_BAT_LEVEL2) {
      LED_ON(BAT_CHECKER_LED);
    } else if(electrical.vsupply < WARN_BAT_LEVEL1) {
      LED_TOGGLE(BAT_CHECKER_LED);
    }
  } else {
    LED_OFF(BAT_CHECKER_LED);
  }
}

