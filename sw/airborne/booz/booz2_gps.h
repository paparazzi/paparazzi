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

#ifndef BOOZ2_GPS_H
#define BOOZ2_GPS_H

#include "std.h"
#include "math/pprz_geodetic_int.h"

struct Booz_gps_state {
  struct EcefCoor_i ecef_pos;    /* pos ECEF in cm        */
  struct EcefCoor_i ecef_vel;    /* speed ECEF in cm/s    */
  struct LlaCoor_i lla_pos;      /* pos LLA               */
  int32_t hmsl;                  /* above mean sea level  */
  uint32_t pacc;                 /* position accuracy     */
  uint32_t sacc;                 /* speed accuracy        */
  uint16_t pdop;                 /* dilution of precision */
  uint8_t  num_sv;               /* number of sat in fix  */
  uint8_t  fix;                  /* status of fix         */
};

extern struct Booz_gps_state booz_gps_state;

extern uint8_t booz_gps_lost_counter; /* updated at 4Hz */


/* UBX NAV SOL */
#define  BOOZ2_GPS_FIX_NONE 0x00
#define  BOOZ2_GPS_FIX_3D   0x03


#include "ubx_protocol.h"

#define GpsFixValid() (booz_gps_state.fix == BOOZ2_GPS_FIX_3D)

#ifdef SITL
extern bool_t booz_gps_available;
#define GPS_LINKChAvailable() (FALSE)
#define GPS_LINKGetch() (TRUE)
#include "nps_sensors.h"

static inline void  booz_gps_feed_value() {
  booz_gps_state.ecef_pos.x = sensors.gps.ecef_pos.x * 100.;
  booz_gps_state.ecef_pos.y = sensors.gps.ecef_pos.y * 100.;
  booz_gps_state.ecef_pos.z = sensors.gps.ecef_pos.z * 100.;
  booz_gps_state.ecef_vel.x = sensors.gps.ecef_vel.x * 100.;
  booz_gps_state.ecef_vel.y = sensors.gps.ecef_vel.y * 100.;
  booz_gps_state.ecef_vel.z = sensors.gps.ecef_vel.z * 100.;
  booz_gps_state.fix = BOOZ2_GPS_FIX_3D;
  booz_gps_available = TRUE;
}

#define Booz2GpsEvent(_sol_available_callback) {	        \
    if (booz_gps_available) {					\
      if (booz_gps_state.fix == BOOZ2_GPS_FIX_3D)               \
        booz_gps_lost_counter = 0;                              \
      _sol_available_callback();				\
      booz_gps_available = FALSE;				\
    }								\
  }
#else /* ! SITL */
#define Booz2GpsEvent(_sol_available_callback) {                    \
    if (GpsBuffer()) {                                              \
      ReadGpsBuffer();                                              \
    }                                                               \
    if (ubx_msg_available) {                                        \
      booz2_gps_read_ubx_message();                                 \
      if (ubx_class == UBX_NAV_ID && ubx_id == UBX_NAV_SOL_ID) {	\
        if (booz_gps_state.fix == BOOZ2_GPS_FIX_3D)                     \
          booz_gps_lost_counter = 0;                                    \
	_sol_available_callback();					\
      }									\
      ubx_msg_available = FALSE;					\
    }									\
  }
#endif

extern void booz2_gps_init(void);
extern void booz2_gps_read_ubx_message(void);


#define __GpsLink(dev, _x) dev##_x
#define _GpsLink(dev, _x)  __GpsLink(dev, _x)
#define GpsLink(_x) _GpsLink(GPS_LINK, _x)

#define GpsBuffer() GpsLink(ChAvailable())
#define ReadGpsBuffer() {                          \
    while (GpsLink(ChAvailable())&&!ubx_msg_available)  \
      ubx_parse(GpsLink(Getch()));                      \
  }

/* UBX parsing - copied from gps_ubx.c */

extern bool_t  ubx_msg_available;
extern uint8_t ubx_msg_buf[];
extern uint8_t ubx_id;
extern uint8_t ubx_class;

extern void ubx_parse( uint8_t c );
extern void ubx_init(void);

static inline void booz_gps_periodic( void ) {
  RunOnceEvery(128, booz_gps_lost_counter++; );
}

#define GpsIsLost() (booz_gps_lost_counter > 20) /* 4Hz -> 5s */


#endif /* BOOZ2_GPS_H */
