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
#include "mcu.h"
#include "sys_time.h"
#include "downlink.h"
#include "subsystems/gps.h"
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
  mcu_init();
  sys_time_init();
  led_init();
  gps_init();
  mcu_int_enable();
}

static inline void main_periodic_task( void ) {

  RunOnceEvery(128, { DOWNLINK_SEND_ALIVE(DefaultChannel, 16, MD5SUM);});
  RunOnceEvery(128, { LED_PERIODIC();});
}

static inline void main_event_task( void ) {
  GpsEvent(on_gps_sol);

}

static void on_gps_sol(void) {

  DOWNLINK_SEND_GPS_INT( DefaultChannel,
			   &gps.ecef_pos.x,
			   &gps.ecef_pos.y,
			   &gps.ecef_pos.z,
			   &gps.lla_pos.lat,
			   &gps.lla_pos.lon,
			   &gps.lla_pos.alt,
			   &gps.ecef_vel.x,
			   &gps.ecef_vel.y,
			   &gps.ecef_vel.z,
			   &gps.pacc,
			   &gps.sacc,
			   &gps.tow,
			   &gps.pdop,
			   &gps.num_sv,
			   &gps.fix);

}
