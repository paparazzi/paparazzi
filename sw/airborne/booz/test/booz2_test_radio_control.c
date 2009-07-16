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

#include "init_hw.h"
#include "sys_time.h"
#include "interrupt_hw.h"

#include "downlink.h"

#include "booz_radio_control.h"

static inline void main_init( void );
static inline void main_periodic_task( void );
static inline void main_event_task( void );
static        void main_on_radio_control_frame( void );
//static        void main_on_radio_control_status_changed( void );

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
  hw_init();
  sys_time_init();
  radio_control_init();
  int_enable();
}

static inline void main_periodic_task( void ) {
  
  RunOnceEvery(51, {/*LED_TOGGLE(2);*/ DOWNLINK_SEND_TIME(&cpu_time_sec);});  

  RunOnceEvery(10, {radio_control_periodic();});

  RunOnceEvery(10, {DOWNLINK_SEND_BOOZ2_RADIO_CONTROL(&radio_control.values[RADIO_CONTROL_ROLL],     \
						      &radio_control.values[RADIO_CONTROL_PITCH],    \
						      &radio_control.values[RADIO_CONTROL_YAW],	     \
						      &radio_control.values[RADIO_CONTROL_THROTTLE], \
						      &radio_control.values[RADIO_CONTROL_MODE],     \
						      &radio_control.status);});
}

static inline void main_event_task( void ) {
 
  RadioControlEvent(main_on_radio_control_frame);
 
}

static void main_on_radio_control_frame( void ) {

  //  RunOnceEvery(10, {DOWNLINK_SEND_RC(RADIO_CONTROL_NB_CHANNEL, radio_control.values);});

}

/*
static void main_on_radio_control_status_changed( void ) {

}
*/
