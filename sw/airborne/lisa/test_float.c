/*
 * $Id$
 *
 * Copyright (C) 2009 Antoine Drouin <poinix@gmail.com>
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

#include <math.h>

#include BOARD_CONFIG
#include "mcu.h"
#include "mcu_periph/sys_time.h"
#include "subsystems/datalink/downlink.h"
#include "led.h"
static inline void main_init( void );
static inline void main_periodic( void );


int main(void) {

  main_init();

  while (1) {
    if (sys_time_check_and_ack_timer(0))
      main_periodic();
  }
  return 0;
}

static inline void main_init( void ) {
  mcu_init();
  sys_time_register_timer((1./PERIODIC_FREQUENCY), NULL);
}

static inline void main_periodic( void ) {
  static float f = 0.1f;
  f += 0.001;
  float f1 = sinf(f);     // ok
  static double d = 0.1;
  d += 0.0025;
  double d1 = sin(d);

  float i = sqrt(f);  // ok
  //float i = powf(f1, f1); // nok
  //float i = atan2(f, f); // ok
  RunOnceEvery(10, {DOWNLINK_SEND_TEST_FORMAT(DefaultChannel, DefaultDevice, &d1, &i);});

  uint16_t  blaaa = f+d;

  RunOnceEvery(10, {DOWNLINK_SEND_BOOT(DefaultChannel, DefaultDevice, &blaaa);});
  LED_PERIODIC();
}



