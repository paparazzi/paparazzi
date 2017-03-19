/*
 * Copyright (C) 2012 Ewoud Smeur <ewoud_smeur@msn.com>
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

/** @file modules/servo_tester/servo_tester.c
 * Module that has a setting to change the input to a servo
 * Call the servo that you want to test SERVO_TEST,
 * and don't use it with the command laws
 */

#include "servo_tester.h"
#include "subsystems/commands.h"
#include "generated/airframe.h"
#include "generated/airframe.h"
#include "subsystems/actuators.h"
#include "generated/modules.h"

int32_t servo_test_val;

bool do_servo_run;
int32_t servo_run_counter;
int32_t servo_run_timer;
int32_t servo_run_step;

#ifndef TIME_PER_DEFLECTION
/** Time in seconds per deflection, before going to the next */
#define TIME_PER_DEFLECTION 0.8
#endif

int32_t servo_mean;

void servo_tester_init()
{
  servo_test_val = SERVO_SERVO_TEST_NEUTRAL;
  do_servo_run = false;
  servo_run_counter = 0;
  servo_run_timer = 0;
  servo_run_step = (SERVO_SERVO_TEST_MAX - SERVO_SERVO_TEST_MIN) / 2 / 8;

  servo_mean = SERVO_SERVO_TEST_NEUTRAL;
}


void servo_tester_periodic()
{

  /*Go up and down with increasing amplitude*/
  if (do_servo_run && (servo_run_counter < 8)) {
    servo_test_val = servo_mean + servo_run_counter * servo_run_step * ((servo_run_counter % 2) * 2 - 1);
    servo_run_timer += 1;

    /*Give time to reach the setpoint*/
    if (servo_run_timer > (TIME_PER_DEFLECTION*SERVO_TESTER_PERIODIC_FREQ)) {
      servo_run_timer = 0;
      servo_run_counter += 1;
    }
  } else if (do_servo_run == true) {
    /*Reset de servo run*/
    do_servo_run = false;
    servo_run_counter = 0;
    servo_test_val = servo_mean;
  }

  Bound(servo_test_val, SERVO_SERVO_TEST_MIN, SERVO_SERVO_TEST_MAX);
  Set_SERVO_TEST_Servo(servo_test_val);
}

