/*
 * Copyright (C) 2010 Antoine Drouin <poinix@gmail.com>
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
 * @file test_actuators_pwm_sin.c
 *
 * Simple test prog for PWM actuators.
 * Move each PWM actuator from 1ms to 2ms.
 */

#include <inttypes.h>

#include "mcu.h"
#include "mcu_periph/sys_time.h"
#include "modules/actuators/actuators_pwm.h"
#include "led.h"

static inline void main_init(void);
static inline void main_periodic(void);

int main(void)
{

  main_init();
  while (1) {
    if (sys_time_check_and_ack_timer(0)) {
      main_periodic();
    }
  };
  return 0;
}

static inline void main_init(void)
{
  mcu_init();
  sys_time_register_timer((1. / PERIODIC_FREQUENCY), NULL);
  ActuatorsPwmInit();
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

  LED_PERIODIC();
}

