/*
 * $Id$
 *
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

#include <inttypes.h>

#include "init_hw.h"
#include "sys_time.h"
#include "firmwares/rotorcraft/actuators/actuators_pwm.h"

static inline void main_init( void );
static inline void main_periodic( void );

int main(void) {

  main_init();
  while (1) {
    if (sys_time_periodic())
      main_periodic();
  };
  return 0;
}

static inline void main_init( void ) {
  hw_init();
  sys_time_init();
  actuators_init();
}

static inline void main_periodic( void ) {
  static float foo = 0.;
  foo += 0.0025;
  int32_t bar = 1500 + 500. * sin(foo);
  //  int32_t bar = 1450;
  //  int32_t bar = 1950;
  actuators_pwm_values[0] = bar;
  actuators_pwm_values[1] = bar;
  actuators_pwm_values[2] = bar;
  actuators_pwm_values[3] = bar;
  actuators_pwm_values[4] = bar;
  actuators_pwm_values[5] = bar;
  actuators_pwm_commit();

  LED_PERIODIC();
}



