/*
 * Copyright (C) 2003-2011 The Paparazzi Team
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

/** @file gps.h
 *  @brief Device independent GPS code
 *
 */

#ifndef GPS_H
#define GPS_H


#include "std.h"
#include "math/pprz_geodetic_int.h"
#include "mcu_periph/uart.h"


/* GPS model specific implementation or sim */
#ifdef GPS_TYPE_H
#include GPS_TYPE_H
#endif

#define GPS_FIX_NONE 0x00
#define GPS_FIX_3D   0x03

#define GpsFixValid() (gps.fix == GPS_FIX_3D)


#ifndef GPS_NB_CHANNELS
#define GPS_NB_CHANNELS 1
#endif

/** Space Vehicle Information */
struct SVinfo {
  uint8_t svid;
  uint8_t flags;
  uint8_t qi;
  uint8_t cno;
  int8_t elev;  ///< deg
  int16_t azim; ///< deg
};

struct GpsState {
  struct EcefCoor_i ecef_pos;    ///< position in ECEF in cm
  struct LlaCoor_i lla_pos;      ///< position in LLA (lat,lon: rad*1e7; alt: mm over ellipsoid)
  struct UtmCoor_i utm_pos;      ///< position in UTM (north,east: cm; alt: mm over ellipsoid)
  int32_t hmsl;                  ///< height above mean sea level in mm
  struct EcefCoor_i ecef_vel;    ///< speed ECEF in cm/s
  struct NedCoor_i ned_vel;      ///< speed NED in cm/s
  int16_t gspeed;                ///< norm of 2d ground speed in cm/s
  int16_t speed_3d;              ///< norm of 3d speed in cm/s
  int32_t course;                ///< GPS heading in rad*1e7
  uint32_t pacc;                 ///< position accuracy
  uint32_t sacc;                 ///< speed accuracy
  uint16_t pdop;                 ///< dilution of precision
  uint8_t num_sv;                ///< number of sat in fix
  uint8_t fix;                   ///< status of fix
  int16_t week;                  ///< GPS week
  uint32_t tow;                  ///< time of week in ms

  uint8_t nb_channels;           ///< Number of scanned satellites
  struct SVinfo svinfos[GPS_NB_CHANNELS];

  uint8_t  lost_counter;         /* updated at 4Hz        */
  uint16_t last_msg_time;
};

struct GpsTimeSync {
  uint32_t t0_tow;      ///< for time sync: time of week in ms for time sync
  int32_t t0_tow_frac;  ///< for time sync: fractional ns remainder of tow [ms], range -500000 .. 500000
  uint32_t t0;          ///< for time sync: hw clock ticks when GPS message is received
};

extern struct GpsState gps;



extern void gps_init(void);

/* GPS model specific init implementation */
extern void gps_impl_init(void);

extern void gps_configure(void);



//TODO
// this is only true for a 512Hz main loop
// needs to work with different main loop frequencies
static inline void gps_periodic( void ) {
  RunOnceEvery(128, gps.lost_counter++; );
}

#define GpsIsLost() (gps.lost_counter > 20) /* 4Hz -> 5s */



#ifdef GPS_TIMESTAMP
#ifndef PCLK
#error unknown PCLK frequency
#endif

extern struct GpsTimeSync gps_time;

uint32_t gps_tow_from_ticks(uint32_t clock_ticks)
{
  uint32_t clock_delta;
  uint32_t time_delta;
  uint32_t itow_now;

  if (clock_ticks < gps_t0) {
    clock_delta = (0xFFFFFFFF - clock_ticks) + gps_time.t0 + 1;
  } else {
    clock_delta = clock_ticks - gps_time.t0;
  }

  time_delta = MSEC_OF_SYS_TICS(clock_delta);

  itow_now = gps_time.t0_tow + time_delta;
  if (itow_now > MSEC_PER_WEEK) itow_now %= MSEC_PER_WEEK;

  return itow_now;
}
#endif


#endif /* GPS_H */
