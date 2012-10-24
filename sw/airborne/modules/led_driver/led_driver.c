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

#include "led_driver.h"

/*
 * simple module to blink LEDs when battery voltage drops below a certain 
 * level, as well as when AHRS is not aligned or when takeoff safety conditions
 * are not met.
 */

/* initialises periodic loop; place more init functions here if expanding driver */
void led_driver_init(void) {
  LED_ON(AHRS_ALIGNER_LED);
  led_driver_periodic();
}


void led_driver_periodic(void) {
  
#ifdef AHRS_ALIGNER_LED
// #ifdef AUTOPILOT_LOBATT_BLINK
  if (radio_control.status == RC_LOST || radio_control.status == RC_REALLY_LOST){
    //RunXTimesEvery(300, 5, 9, {LED_TOGGLE(AHRS_ALIGNER_LED);});
    RunXTimesEvery(0, 60, 5, 7, {LED_TOGGLE(AHRS_ALIGNER_LED);});
    RunXTimesEvery(130, 130, 10, 6, {LED_TOGGLE(AHRS_ALIGNER_LED);});
    }
//   TODO: Check for erroneous IMU data
//   else if (ahrs_aligner.status == AHRS_ALIGNER_FROZEN){
//     //RunXTimesEvery(0, 120, 5, 4, {LED_TOGGLE(AHRS_ALIGNER_LED);});
//     RunXTimesEvery(5, 200, 10, 20, {LED_ON(AHRS_ALIGNER_LED);});
//     RunXTimesEvery(0, 200, 10, 20, {LED_OFF(AHRS_ALIGNER_LED);});
//     }
//   else if (autopilot_first_boot){
//     //RunXTimesEvery(0, 120, 5, 4, {LED_TOGGLE(AHRS_ALIGNER_LED);});
//     RunXTimesEvery(5, 120, 10, 2, {LED_ON(AHRS_ALIGNER_LED);});
//     RunXTimesEvery(0, 120, 10, 2, {LED_OFF(AHRS_ALIGNER_LED);});
//     }
//   else if (autopilot_safety_violation_mode){
//     //RunXTimesEvery(0, 240, 20, 2, {LED_TOGGLE(AHRS_ALIGNER_LED);});
//     RunXTimesEvery(20, 240, 40, 1, {LED_ON(AHRS_ALIGNER_LED);});
//     RunXTimesEvery(0, 240, 40, 1, {LED_OFF(AHRS_ALIGNER_LED);});
//     }
//   else if (autopilot_safety_violation_throttle){
//     //RunXTimesEvery(0, 240, 20, 4, {LED_TOGGLE(AHRS_ALIGNER_LED);});
//     RunXTimesEvery(20, 240, 40, 2, {LED_ON(AHRS_ALIGNER_LED);});
//     RunXTimesEvery(0, 240, 40, 2, {LED_OFF(AHRS_ALIGNER_LED);});
//     }
//   else if (autopilot_safety_violation_roll){
//     //RunXTimesEvery(0, 240, 20, 6, {LED_TOGGLE(AHRS_ALIGNER_LED);});
//     RunXTimesEvery(20, 240, 40, 3, {LED_ON(AHRS_ALIGNER_LED);});
//     RunXTimesEvery(0, 240, 40, 3, {LED_OFF(AHRS_ALIGNER_LED);});
//     }
//   else if (autopilot_safety_violation_pitch){
//     //RunXTimesEvery(0, 240, 20, 8, {LED_TOGGLE(AHRS_ALIGNER_LED);});
//     RunXTimesEvery(20, 240, 40, 4, {LED_ON(AHRS_ALIGNER_LED);});
//     RunXTimesEvery(0, 240, 40, 4, {LED_OFF(AHRS_ALIGNER_LED);});
//     }
//   else if (autopilot_safety_violation_yaw){
//     //RunXTimesEvery(0, 240, 20,10, {LED_TOGGLE(AHRS_ALIGNER_LED);});
//     RunXTimesEvery(20, 240, 40, 5, {LED_ON(AHRS_ALIGNER_LED);});
//     RunXTimesEvery(0, 240, 40, 5, {LED_OFF(AHRS_ALIGNER_LED);});
//     }
//   else if (autopilot_safety_violation){
//     RunOnceEvery(5, {LED_TOGGLE(AHRS_ALIGNER_LED);});
//     }
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
// #endif
}




