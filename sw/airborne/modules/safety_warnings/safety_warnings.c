/*
 * $Id: $
 *
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
#include "led.h"
#include "generated/airframe.h"
#include "subsystems/electrical.h"
#include "subsystems/radio_control.h"
#include "autopilot.h"
#include "subsystems/ahrs/ahrs_aligner.h"

#include "safety_warnings.h"

/*
 * simple module to blink LEDs when battery voltage drops below a certain 
 * level, as well as when AHRS is not aligned or when takeoff safety conditions
 * are not met.
 */

/* initialises periodic loop; place more init functions here if expanding driver */
void safety_warnings_init(void) {
  LED_ON(AHRS_ALIGNER_LED);
  safety_warnings_periodic();
}


void safety_warnings_periodic(void) {
  
#ifdef AHRS_ALIGNER_LED
  if (radio_control.status == RC_LOST || radio_control.status == RC_REALLY_LOST){
    RunXTimesEvery(0, 60, 5, 7, {LED_TOGGLE(AHRS_ALIGNER_LED);});
    RunXTimesEvery(130, 130, 10, 6, {LED_TOGGLE(AHRS_ALIGNER_LED);});
    }
  #ifdef MIN_BAT_LEVEL
  else if (electrical.vsupply < (MIN_BAT_LEVEL * 10)){
    RunOnceEvery(20, {LED_TOGGLE(AHRS_ALIGNER_LED);});
    }
  else if (electrical.vsupply < ((MIN_BAT_LEVEL + 0.5) * 10)){
    RunXTimesEvery(0, 300, 10, 10, {LED_TOGGLE(AHRS_ALIGNER_LED);});
    }
  #endif
  else {
    LED_ON(AHRS_ALIGNER_LED);
    }
#endif
}
