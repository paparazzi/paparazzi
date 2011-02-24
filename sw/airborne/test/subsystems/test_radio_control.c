/*
 * $Id$
 *
 * Copyright (C) 2008-2010 The Paparazzi Team
 *
 * This file is part of Paparazzi.
 *
 * Paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * Paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 */

#include <inttypes.h>

#include "mcu.h"
#include "sys_time.h"
#include "interrupt_hw.h"
#include "mcu_periph/uart.h"

#include "downlink.h"

#include "subsystems/radio_control.h"

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
  mcu_init();
  sys_time_init();
  radio_control_init();
  mcu_int_enable();
}

extern uint32_t debug_len;

static inline void main_periodic_task( void ) {

  RunOnceEvery(51, {
    /*LED_TOGGLE(2);*/
    uint32_t blaaa= cpu_time_sec;
    DOWNLINK_SEND_TIME(DefaultChannel, &blaaa);
  });

  RunOnceEvery(10, {radio_control_periodic_task();});

  int16_t foo = 0;//RC_PPM_SIGNED_TICS_OF_USEC(2050-1500);
  RunOnceEvery(10,
    {DOWNLINK_SEND_BOOZ2_RADIO_CONTROL(DefaultChannel,	\
				       &radio_control.values[RADIO_ROLL], \
				       &radio_control.values[RADIO_PITCH], \
				       &radio_control.values[RADIO_YAW], \
				       &radio_control.values[RADIO_THROTTLE], \
				       &radio_control.values[RADIO_MODE], \
				       &foo,				\
				       &radio_control.status);});
#ifdef RADIO_CONTROL_TYPE_PPM
  RunOnceEvery(10,
	       {uint8_t blaa = 0; DOWNLINK_SEND_PPM(DefaultChannel,&blaa, 8, booz_radio_control_ppm_pulses);});
#endif

  LED_PERIODIC();
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
