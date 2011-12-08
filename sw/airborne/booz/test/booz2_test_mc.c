/*
 * $Id$
 *
 * Copyright (C) 2008-2009 Antoine Drouin <poinix@gmail.com>
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

#include "mcu.h"
#include "sys_time.h"
#include "led.h"
#include "mcu_periph/uart.h"

#include "messages.h"
#include "subsystems/datalink/downlink.h"

#include "commands.h"
#include "mcu_periph/i2c.h"
#include "firmwares/rotorcraft/actuators.h"

static inline void main_init( void );
static inline void main_periodic_task( void );
static inline void main_event_task( void );

uint32_t t0, t1, diff;

int main( void ) {
  main_init();
  while(1) {
    if (sys_time_periodic())
      main_periodic_task();
    main_event_task();
  }
  return 0;
}

static inline void main_init( void ) {
  mcu_init();
  sys_time_init();
  led_init();
  uart1_init_tx();

  i2c_init();
  actuators_init();

  mcu_int_enable();
}

static inline void main_periodic_task( void ) {
  RunOnceEvery(50, {LED_TOGGLE(2); DOWNLINK_SEND_TIME(&cpu_time_sec);});

  Actuator(SERVO_FRONT) = 20;
  Actuator(SERVO_BACK)  = 20;
  Actuator(SERVO_RIGHT) = 20;
  Actuator(SERVO_LEFT)  = 20;
  ActuatorsCommit();

}

static inline void main_event_task( void ) {

}
