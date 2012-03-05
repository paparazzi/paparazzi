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


#include "mcu.h"
#include "mcu_periph/sys_time.h"
#include "firmwares/rotorcraft/commands.h"
#include "actuators.h"
#include "subsystems/datalink/downlink.h"
#include "led.h"

#include "actuators/actuators_asctec.h"

static inline void main_init( void );
static inline void main_periodic_task( void );
static inline void main_event_task( void );

//static uint8_t i2c_done;

int main(void) {
  main_init();

  while(1) {
    if (sys_time_check_and_ack_timer(0))
      main_periodic_task();
    main_event_task();
  }

  return 0;
}

static inline void main_init( void ) {
  mcu_init();
  sys_time_register_timer((1./PERIODIC_FREQUENCY), NULL);
  actuators_init();
}



static inline void main_periodic_task( void ) {

  static uint16_t i = 0;

  RunOnceEvery(100, {
      LED_TOGGLE(3);
      DOWNLINK_SEND_ALIVE(DefaultChannel, DefaultDevice, 16, MD5SUM);
    });


#if 0
  if (i==1) {
    actuators_asctec.cur_addr = BACK;
    actuators_asctec.cmd = TEST;
  }
#endif
#if 0
  if (i==1) {
    actuators_asctec.cur_addr = LEFT;
    actuators_asctec.new_addr = BACK;
    actuators_asctec.cmd = SET_ADDR;
  }
#endif
#if 0
  if (i==1) {
    actuators_asctec.cur_addr = BACK;
    actuators_asctec.cmd = REVERSE;
  }
#endif
  i++;

  if (i>1000) {
    /* set actuators     */
    commands[COMMAND_PITCH] = 0;
    commands[COMMAND_ROLL] = 0;
    commands[COMMAND_YAW] = 20;
    commands[COMMAND_THRUST] = 0;
    // actuators_set(TRUE);
    actuators_set(FALSE);
  }
  LED_PERIODIC();

}



static inline void main_event_task( void ) {

}
