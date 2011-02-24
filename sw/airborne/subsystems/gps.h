/*
 * Copyright (C) 2008-2011 The Paparazzi Team
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

#ifndef GPS_H
#define GPS_H

#include "std.h"
#include "math/pprz_geodetic_int.h"
#include "mcu_periph/uart.h"


/* GPS model specific implementation */
#ifdef GPS_TYPE_H
#include GPS_TYPE_H
#endif

/** Number of scanned satellites */
extern uint8_t gps_nb_channels;

/** Space Vehicle Information */
struct svinfo {
  uint8_t svid;
  uint8_t flags;
  uint8_t qi;
  uint8_t cno;
  int8_t elev; /** deg */
  int16_t azim; /** deg */
};

extern struct svinfo gps_svinfos[GPS_NB_CHANNELS];

struct GpsState {
  struct EcefCoor_i ecef_pos;    /* pos ECEF in cm        */
  struct EcefCoor_i ecef_vel;    /* speed ECEF in cm/s    */
  struct LlaCoor_i lla_pos;      /* pos LLA               */
  int32_t hmsl;                  /* above mean sea level  */
  uint32_t pacc;                 /* position accuracy     */
  uint32_t sacc;                 /* speed accuracy        */
  uint16_t pdop;                 /* dilution of precision */
  uint8_t  num_sv;               /* number of sat in fix  */
  uint8_t  fix;                  /* status of fix         */
  uint32_t tow;                  /* time of week in 1e-2s */

  uint8_t  lost_counter;         /* updated at 4Hz        */
};

extern struct GpsState gps;


/* GPS model specific init implementation */
extern void gps_impl_init(void);


#define  GPS_FIX_NONE 0x00
#define  GPS_FIX_3D   0x03

#define GpsFixValid() (gps.fix == GPS_FIX_3D)

/*
 * This part is used by the simulator to feed simulated data
 *
 */
#ifdef SITL

extern bool_t gps_available;
#define GPS_LINKChAvailable() (FALSE)
#define GPS_LINKGetch() (TRUE)
#include "nps_sensors.h"
#include "generated/flight_plan.h"

static inline void  gps_feed_value() {
  gps.ecef_pos.x = sensors.gps.ecef_pos.x * 100.;
  gps.ecef_pos.y = sensors.gps.ecef_pos.y * 100.;
  gps.ecef_pos.z = sensors.gps.ecef_pos.z * 100.;
  gps.ecef_vel.x = sensors.gps.ecef_vel.x * 100.;
  gps.ecef_vel.y = sensors.gps.ecef_vel.y * 100.;
  gps.ecef_vel.z = sensors.gps.ecef_vel.z * 100.;
  gps.lla_pos.lat = DegOfRad(sensors.gps.lla_pos.lat) * 1e7;
  gps.lla_pos.lon = DegOfRad(sensors.gps.lla_pos.lon) * 1e7;
  gps.lla_pos.alt = sensors.gps.lla_pos.alt * 100.;
  gps.hmsl        = sensors.gps.hmsl * 100.;
  gps.fix = GPS_FIX_3D;
  gps_available = TRUE;
}

#define GpsEvent(_sol_available_callback) {			\
    if (gps_available) {					\
      if (gps.fix == GPS_FIX_3D)               \
        gps.lost_counter = 0;			\
      _sol_available_callback();				\
      gps_available = FALSE;				\
    }								\
  }
#else /* ! SITL */
/*
 * This part is used by the autopilot to read data from a uart
 *
 */


#define __GpsLink(dev, _x) dev##_x
#define _GpsLink(dev, _x)  __GpsLink(dev, _x)
#define GpsLink(_x) _GpsLink(GPS_LINK, _x)

#define GpsBuffer() GpsLink(ChAvailable())




#endif /* !SITL */


extern void gps_init(void);

static inline void gps_periodic( void ) {
  RunOnceEvery(128, gps.lost_counter++; );
}

#define GpsIsLost() (gps.lost_counter > 20) /* 4Hz -> 5s */




#endif /* GPS_H */
