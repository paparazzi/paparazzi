/*
 * Copyright (C) 2015 AggieAir, A Remote Sensing Unmanned Aerial System for Scientific Applications
 * Utah State University, http://aggieair.usu.edu/
 *
 * Michal Podhradsky (michal.podhradsky@aggiemail.usu.edu)
 * Calvin Coopmans (c.r.coopmans@ieee.org)
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
 * @file chibios_test_actuators_pwm_sin.c
 *
 * Simple test prog for PWM actuators.
 * Move each PWM actuator from 1ms to 2ms.
 */

/* ChibiOS includes */
#include "ch.h"


#include <inttypes.h>

#include "mcu.h"
#include "mcu_periph/sys_time.h"
#include "subsystems/actuators/actuators_pwm.h"
#include "led.h"

static inline void main_periodic(void);

/*
 * Red LEDs blinker thread, times are in milliseconds.
 */
static THD_WORKING_AREA(waThdBlinker, 128);
static void ThdBlinker(void *arg) {

  (void)arg;
  chRegSetThreadName("blinker");
  while (TRUE) {
#ifdef SYS_TIME_LED
    LED_TOGGLE(SYS_TIME_LED);
#endif
    chThdSleepMilliseconds(500);
  }
}


int main(void)
{
  mcu_init();
  ActuatorsPwmInit();

  /*
   * Creates the blinker thread.
   */
  chThdCreateStatic(waThdBlinker, sizeof(waThdBlinker), NORMALPRIO, ThdBlinker, NULL);

  while (TRUE) {
    main_periodic();
    sys_time_usleep(1000000/PERIODIC_FREQUENCY);
  }

  return 0;
}



static inline void main_periodic(void)
{
  static float foo = 0.;
  foo += 0.0025;
  int32_t bar = 1500 + 500. * sin(foo);
  for (int i = 0; i < ACTUATORS_PWM_NB; i++) {
    ActuatorPwmSet(i, bar);
  }
  ActuatorsPwmCommit();
}

