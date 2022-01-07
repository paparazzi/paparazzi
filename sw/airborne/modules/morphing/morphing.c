/*
 * Copyright (C) 2021 Murat Bronz
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

/** @file modules/morphing/morphing.c
 *
 * Geometry morphing of a vehicle with the help of servo(s).
 */

#include "morphing.h"
#include "modules/core/commands.h"
#include "generated/airframe.h"
#include "modules/actuators/actuators.h"
#include "generated/modules.h"

// #include "pprzlink/messages.h"
#include "modules/datalink/downlink.h"

int32_t servo_1_val;
int32_t servo_2_val;

float morph_command_1;
float morph_command_2;
int32_t counter;
bool morph_do_dance;
float morph_dance_period;

#ifndef TIME_PER_DEFLECTION
/** Time in seconds per deflection, before going to the next */
#define TIME_PER_DEFLECTION 0.8
#endif

int32_t servo_mean;

void morphing_init()
{

  servo_1_val = SERVO_ARMS1_NEUTRAL;
  servo_2_val = SERVO_ARMS2_NEUTRAL;

  morph_command_1 = 0.;
  morph_command_2 = 0.;

}


void morphing_periodic()
{

  if (morph_do_dance){
    float command_dance = sinf( (morph_dance_period*counter) / MORPHING_PERIODIC_FREQ);
    counter += 1;
    morph_command_1 = command_dance;
    morph_command_2 = -command_dance;

    // Easy debug message :
    // float msg[] = { command_dance };
    // DOWNLINK_SEND_PAYLOAD_FLOAT(DefaultChannel, DefaultDevice, 1, msg);
  }



  if (morph_command_1 >= 0) {
    servo_1_val = SERVO_ARMS1_NEUTRAL + (SERVO_ARMS1_MAX - SERVO_ARMS1_NEUTRAL) * morph_command_1 ;
  } else {
    servo_1_val = SERVO_ARMS1_NEUTRAL - (SERVO_ARMS1_MIN - SERVO_ARMS1_NEUTRAL) * morph_command_1 ;
  }

  if (morph_command_2 >= 0) {
    servo_2_val = SERVO_ARMS2_NEUTRAL + (SERVO_ARMS2_MAX - SERVO_ARMS2_NEUTRAL) * morph_command_2 ;
  } else {
    servo_2_val = SERVO_ARMS2_NEUTRAL - (SERVO_ARMS2_MIN - SERVO_ARMS2_NEUTRAL) * morph_command_2 ;
  }


  Bound(servo_1_val, SERVO_ARMS1_MIN, SERVO_ARMS1_MAX);
  Bound(servo_2_val, SERVO_ARMS2_MIN, SERVO_ARMS2_MAX);
  Set_ARMS1_Servo(servo_1_val);
  Set_ARMS2_Servo(servo_2_val);
}