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

#include "std.h"
#include "init_hw.h"
#include "sys_time.h"
#include "downlink.h"
#include "booz_gps.h"
#include "interrupt_hw.h"

static inline void main_init( void );
static inline void main_periodic_task( void );
static inline void main_event_task( void );

static void on_gps_sol(void);

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
  led_init();
  booz_gps_init();
  int_enable();
}

static inline void main_periodic_task( void ) {

  RunOnceEvery(128, { DOWNLINK_SEND_ALIVE(DefaultChannel, 16, MD5SUM);});
  RunOnceEvery(128, { LED_PERIODIC();});
}

static inline void main_event_task( void ) {
  BoozGpsEvent(on_gps_sol);
  
}

static void on_gps_sol(void) {
 
  DOWNLINK_SEND_BOOZ2_GPS( DefaultChannel,		
			   &booz_gps_state.ecef_pos.x,
			   &booz_gps_state.ecef_pos.y,
			   &booz_gps_state.ecef_pos.z,
			   &booz_gps_state.lla_pos.lat,
			   &booz_gps_state.lla_pos.lon,
			   &booz_gps_state.lla_pos.alt,
			   &booz_gps_state.ecef_vel.x,
			   &booz_gps_state.ecef_vel.y,
			   &booz_gps_state.ecef_vel.z,
			   &booz_gps_state.pacc,
			   &booz_gps_state.sacc,
			   &booz_gps_state.tow,
			   &booz_gps_state.pdop,
			   &booz_gps_state.num_sv,
			   &booz_gps_state.fix);

}
