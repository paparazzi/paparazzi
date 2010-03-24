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
#include "actuators/booz_actuators_pwm.h"

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
  booz_actuators_pwm_hw_init();
}

static inline void main_periodic( void ) {
  //  static float foo = 0.;
  //  foo += 0.001;
  //  int32_t bar = 9600. * sin(foo);
  static int32_t foo = 10;
  static int32_t bar = 0;
  bar += foo;
  if (bar > 9600) { foo = -foo; bar = 9600;}
  if (bar < -9600) { foo = -foo; bar = -9600;}
  booz_actuators_pwm_values[0] = 3375 + bar * 1125 / 9600;
  booz_actuators_pwm_commit();
}



