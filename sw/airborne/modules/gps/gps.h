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
#include "math/pprz_geodetic_float.h"

#include "mcu_periph/sys_time.h"

#define GPS_FIX_NONE 0x00     ///< No GPS fix
#define GPS_FIX_2D   0x02     ///< 2D GPS fix
#define GPS_FIX_3D   0x03     ///< 3D GPS fix
#define GPS_FIX_DGPS 0x04     ///< DGPS fix
#define GPS_FIX_RTK  0x05     ///< RTK GPS fix

#define GpsFixValid() (gps.fix >= GPS_FIX_3D)
#if USE_GPS
#define GpsIsLost() !GpsFixValid()
#endif

#define GPS_VALID_POS_ECEF_BIT 0
#define GPS_VALID_POS_LLA_BIT  1
#define GPS_VALID_POS_UTM_BIT  2
#define GPS_VALID_VEL_ECEF_BIT 3
#define GPS_VALID_VEL_NED_BIT  4
#define GPS_VALID_HMSL_BIT     5
#define GPS_VALID_COURSE_BIT   6

#ifndef GPS_NB_CHANNELS
#define GPS_NB_CHANNELS 40
#endif

#define GPS_MODE_AUTO 0
#define GPS_MODE_PRIMARY 1
#define GPS_MODE_SECONDARY 2

#ifndef MULTI_GPS_MODE
#define MULTI_GPS_MODE GPS_MODE_AUTO
#endif

/* expand GpsId(PRIMARY_GPS) to e.g. GPS_UBX_ID */
#define __GpsId(_x) _x ## _ID
#define _GpsId(_x) __GpsId(_x)
#define GpsId(_x) _GpsId(_x)


extern uint8_t multi_gps_mode;

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
  uint8_t valid_fields;          ///< bitfield indicating valid fields (GPS_VALID_x_BIT)
  uint8_t comp_id;               ///< id of current gps

  struct EcefCoor_i ecef_pos;    ///< position in ECEF in cm
  struct LlaCoor_i lla_pos;      ///< position in LLA (lat,lon: deg*1e7; alt: mm over ellipsoid)
  struct UtmCoor_i utm_pos;      ///< position in UTM (north,east: cm; alt: mm over MSL)
  int32_t hmsl;                  ///< height above mean sea level (MSL) in mm
  struct EcefCoor_i ecef_vel;    ///< speed ECEF in cm/s
  struct NedCoor_i ned_vel;      ///< speed NED in cm/s
  uint16_t gspeed;               ///< norm of 2d ground speed in cm/s
  uint16_t speed_3d;             ///< norm of 3d speed in cm/s
  int32_t course;                ///< GPS course over ground in rad*1e7, [0, 2*Pi]*1e7 (CW/north)
  uint32_t pacc;                 ///< position accuracy in cm
  uint32_t hacc;                 ///< horizontal accuracy in cm
  uint32_t vacc;                 ///< vertical accuracy in cm
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

/** data structures for GPS with RTK capabilities */
struct GpsRelposNED {
  uint32_t iTOW;
  uint16_t refStationId;
  int32_t relPosN;
  int32_t relPosE;
  int32_t relPosD;
  int8_t relPosHPN;
  int8_t relPosHPE;
  int8_t relPosHPD;
  uint32_t accN;
  uint32_t accE;
  uint32_t accD;
  uint8_t carrSoln;
  uint8_t relPosValid;
  uint8_t diffSoln;
  uint8_t gnssFixOK;
};

struct RtcmMan {
  uint16_t RefStation;
  uint16_t MsgType; // Counter variables to count the number of Rtcm msgs in the input stream(for each msg type)
  uint32_t Cnt105;
  uint32_t Cnt177;
  uint32_t Cnt187; // Counter variables to count the number of messages that failed Crc Check
  uint32_t Crc105;
  uint32_t Crc177;
  uint32_t Crc187;
};

/** global GPS state */
extern struct GpsState gps;

#ifdef GPS_TYPE_H
#include GPS_TYPE_H
#endif
#ifdef GPS_SECONDARY_TYPE_H
#include GPS_SECONDARY_TYPE_H
#endif

/** initialize the global GPS state */
extern void gps_init(void);

/** GPS packet injection (default empty) */
extern void gps_inject_data(uint8_t packet_id, uint8_t length, uint8_t *data);

extern void gps_parse_GPS_INJECT(uint8_t *buf);
extern void gps_parse_RTCM_INJECT(uint8_t *buf);

/** GPS timeout in seconds */
#ifndef GPS_TIMEOUT
#define GPS_TIMEOUT 2
#endif

static inline bool gps_has_been_good(void)
{
  static bool gps_had_valid_fix = false;
  if (GpsFixValid()) {
    gps_had_valid_fix = true;
  }
  return gps_had_valid_fix;
}


/** Periodic GPS check.
 * Marks GPS as lost when no GPS message was received for GPS_TIMEOUT seconds
 */
extern void gps_periodic_check(struct GpsState *gps_s);


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

/**
 * Convenience function to get utm position in float from GPS structure.
 * Beware that altitude is initialized to zero but not set to the correct value
 * @param[in] gps_s pointer to the gps structure
 * @param[in] zone set the utm zone in which the position should be computed, 0 to try to get it automatically from lla position
 * @return utm position in float, altitude hmsl.
 */
extern struct UtmCoor_f utm_float_from_gps(struct GpsState *gps_s, uint8_t zone);

/**
 * Convenience function to get utm position in int from GPS structure.
 * Beware that altitude is initialized to zero but not set to the correct value
 * @param[in] gps_s pointer to the gps structure
 * @param[in] zone set the utm zone in which the position should be computed, 0 to try to get it automatically from lla position
 * @return utm position in fixed point (cm), altitude hmsl (mm).
 */
extern struct UtmCoor_i utm_int_from_gps(struct GpsState *gps_s, uint8_t zone);

/**
 * Number of days since navigation epoch (6 January 1980)
 * @param[in] year current year
 * @param[in] month current month
 * @param[in] day current day
 * @return number of days since navigation epoch
 */
extern uint16_t gps_day_number(uint16_t year, uint8_t month, uint8_t day);

/**
 * Number of weeks since navigation epoch (6 January 1980)
 * @param[in] year current year
 * @param[in] month current month
 * @param[in] day current day
 * @return number of weeks since navigation epoch
 */
extern uint16_t gps_week_number(uint16_t year, uint8_t month, uint8_t day);

#endif /* GPS_H */
