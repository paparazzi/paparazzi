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
 *  @brief Device independent GPS code (interface)
 *
 */

#ifndef GPS_H
#define GPS_H


#include "std.h"
#include "math/pprz_geodetic_int.h"

#include "mcu_periph/sys_time.h"

/* GPS model specific implementation or sim */
#ifdef GPS_TYPE_H
#include GPS_TYPE_H
#endif

#define GPS_FIX_NONE 0x00
#define GPS_FIX_2D   0x02
#define GPS_FIX_3D   0x03

#define GpsFixValid() (gps.fix == GPS_FIX_3D)


#ifndef GPS_NB_CHANNELS
#define GPS_NB_CHANNELS 1
#endif

/** data structure for Space Vehicle Information of a single satellite */
struct SVinfo {
  uint8_t svid;  ///< Satellite ID
  uint8_t flags; ///< bitfield with GPS receiver specific flags
  uint8_t qi;    ///< quality bitfield (GPS receiver specific)
  uint8_t cno;   ///< Carrier to Noise Ratio (Signal Strength) in dbHz
  int8_t elev;   ///< elevation in deg
  int16_t azim;  ///< azimuth in deg
};

/** data structure for GPS information */
struct GpsState {
  struct EcefCoor_i ecef_pos;    ///< position in ECEF in cm
  struct LlaCoor_i lla_pos;      ///< position in LLA (lat,lon: deg*1e7; alt: mm over ellipsoid)
  struct UtmCoor_i utm_pos;      ///< position in UTM (north,east: cm; alt: mm over ellipsoid)
  int32_t hmsl;                  ///< height above mean sea level in mm
  struct EcefCoor_i ecef_vel;    ///< speed ECEF in cm/s
  struct NedCoor_i ned_vel;      ///< speed NED in cm/s
  uint16_t gspeed;               ///< norm of 2d ground speed in cm/s
  uint16_t speed_3d;             ///< norm of 3d speed in cm/s
  int32_t course;                ///< GPS course over ground in rad*1e7, [0, 2*Pi]*1e7 (CW/north)
  uint32_t pacc;                 ///< position accuracy in cm
  uint32_t sacc;                 ///< speed accuracy in cm/s
  uint32_t cacc;                 ///< course accuracy in rad*1e7
  uint16_t pdop;                 ///< position dilution of precision scaled by 100
  uint8_t num_sv;                ///< number of sat in fix
  uint8_t fix;                   ///< status of fix
  uint16_t week;                 ///< GPS week
  uint32_t tow;                  ///< GPS time of week in ms

  uint8_t nb_channels;           ///< Number of scanned satellites
  struct SVinfo svinfos[GPS_NB_CHANNELS]; ///< holds information from the Space Vehicles (Satellites)

  uint32_t last_3dfix_ticks;     ///< cpu time ticks at last valid 3D fix
  uint32_t last_3dfix_time;      ///< cpu time in sec at last valid 3D fix
  uint32_t last_msg_ticks;       ///< cpu time ticks at last received GPS message
  uint32_t last_msg_time;        ///< cpu time in sec at last received GPS message
  uint16_t reset;                ///< hotstart, warmstart, coldstart
};

/** data structure for GPS time sync */
struct GpsTimeSync {
  uint32_t t0_tow;      ///< GPS time of week in ms from last message
  int32_t t0_tow_frac;  ///< fractional ns remainder of tow [ms], range -500000 .. 500000
  uint32_t t0_ticks;    ///< hw clock ticks when GPS message is received
};

/** global GPS state */
extern struct GpsState gps;

/** initialize the global GPS state */
extern void gps_init(void);

/* GPS model specific init implementation */
extern void gps_impl_init(void);


/** GPS timeout in seconds */
#ifndef GPS_TIMEOUT
#define GPS_TIMEOUT 2
#endif

static inline bool_t GpsIsLost(void)
{
  if (gps.fix == GPS_FIX_3D) {
    return FALSE;
  }
  return TRUE;
}

static inline bool_t gps_has_been_good(void)
{
  static bool_t gps_had_valid_fix = FALSE;
  if (GpsFixValid()) {
    gps_had_valid_fix = TRUE;
  }
  return gps_had_valid_fix;
}


/** Periodic GPS check.
 * Marks GPS as lost when no GPS message was received for GPS_TIMEOUT seconds
 */
extern void gps_periodic_check(void);

/**
 * GPS Reset
 * @todo this still needs to call gps specific stuff
 */
#define gps_Reset(_val) {                               \
  }


/*
 * For GPS time synchronizaiton...
 */
extern struct GpsTimeSync gps_time_sync;

/**
 * Convert time in sys_time ticks to GPS time of week.
 * The resolution is sys_time.resolution
 * @return GPS tow in ms
 */
extern uint32_t gps_tow_from_sys_ticks(uint32_t sys_ticks);

#endif /* GPS_H */
