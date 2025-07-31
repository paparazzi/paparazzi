/*
 * Copyright (C) 2025 Fabien-B <fabien-b@github.com>
 *
 * This file is part of paparazzi
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
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */

/** @file "modules/gps/gps_uavcan.c"
 * @author Fabien-B <fabien-b@github.com>
 * UAVCAN gps module. Handle uavcan.equipment.gnss.Fix2 message (1063).
 */

#include "modules/gps/gps_uavcan.h"
#include "uavcan/uavcan.h"
#include "core/abi.h"
#include "uavcan.equipment.gnss.Fix2.h"
#include "gps.h"
#include "math/pprz_geodetic_int.h"
#include "math/pprz_geodetic_float.h"
#include "math/pprz_geodetic_double.h"
#include "std.h"



#define SECS_IN_WEEK (7*24*3600000)

static uavcan_event gps_uavcan_ev;

static void gps_uavcan_cb(struct uavcan_iface_t *iface __attribute__((unused)), CanardRxTransfer *transfer) {
  // current timestamp
  uint32_t now_ts = get_sys_time_usec();

  struct uavcan_equipment_gnss_Fix2 msg;
  if(uavcan_equipment_gnss_Fix2_decode(transfer, &msg)) {
    return;   // decode error
  }

  struct GpsState state;

  // uint8_t comp_id;               ///< id of current gps
#if GPS_UAVCAN_USE_NODE_ID
  state.comp_id = transfer->source_node_id;
#else
  state.comp_id = GPS_UAVCAN_ID;
#endif

  // uint32_t last_msg_ticks;       ///< cpu time ticks at last received GPS message
  // uint32_t last_msg_time;        ///< cpu time in sec at last received GPS message
  state.last_msg_ticks = sys_time.nb_sec_rem;
  state.last_msg_time = sys_time.nb_sec;

  // uint8_t fix;                   ///< status of fix
  switch (msg.status)
  {
  case UAVCAN_EQUIPMENT_GNSS_FIX2_STATUS_NO_FIX:
    state.fix = GPS_FIX_NONE;
    break;
  case UAVCAN_EQUIPMENT_GNSS_FIX2_STATUS_TIME_ONLY:
    state.fix = GPS_FIX_NONE;
    break;
  case UAVCAN_EQUIPMENT_GNSS_FIX2_STATUS_2D_FIX:
    state.fix = GPS_FIX_2D;
    break;
  case UAVCAN_EQUIPMENT_GNSS_FIX2_STATUS_3D_FIX:
    if(msg.mode == UAVCAN_EQUIPMENT_GNSS_FIX2_MODE_DGPS) {
      state.fix = GPS_FIX_DGPS;
    } else if(msg.mode == UAVCAN_EQUIPMENT_GNSS_FIX2_MODE_RTK) {
      state.fix = GPS_FIX_RTK;
    } else if(msg.mode == UAVCAN_EQUIPMENT_GNSS_FIX2_MODE_PPP) {
      state.fix = GPS_FIX_3D; // TODO add PPP mode to paparazzi ?
    } else {
      state.fix = GPS_FIX_3D;
    }
    // uint32_t last_3dfix_ticks;     ///< cpu time ticks at last valid 3D fix
    // uint32_t last_3dfix_time;      ///< cpu time in sec at last valid 3D fix
    state.last_3dfix_ticks = sys_time.nb_sec_rem;
    state.last_3dfix_time = sys_time.nb_sec;
    break;
  }
  

  if(msg.ecef_position_velocity.len > 0) {
    // struct EcefCoor_i ecef_pos;    ///< position in ECEF in cm
    state.ecef_pos.x = msg.ecef_position_velocity.data[0].position_xyz_mm[0] / 10;
    state.ecef_pos.y = msg.ecef_position_velocity.data[0].position_xyz_mm[1] / 10;
    state.ecef_pos.z = msg.ecef_position_velocity.data[0].position_xyz_mm[2] / 10;
    SetBit(state.valid_fields, GPS_VALID_POS_ECEF_BIT);

    // struct EcefCoor_i ecef_vel;    ///< speed ECEF in cm/s
    state.ecef_vel.x = msg.ecef_position_velocity.data[0].velocity_xyz[0] * 100;
    state.ecef_vel.y = msg.ecef_position_velocity.data[0].velocity_xyz[1] * 100;
    state.ecef_vel.z = msg.ecef_position_velocity.data[0].velocity_xyz[2] * 100;
    SetBit(state.valid_fields, GPS_VALID_VEL_ECEF_BIT);

    // TODO handle msg.ecef_position_velocity.data[0].covariance
  }

  // struct LlaCoor_i lla_pos;      ///< position in LLA (lat,lon: deg*1e7; alt: mm over ellipsoid)
  state.lla_pos.lat = msg.latitude_deg_1e8 / 10;
  state.lla_pos.lon = msg.longitude_deg_1e8 / 10;
  state.lla_pos.alt = msg.height_ellipsoid_mm;
  SetBit(state.valid_fields, GPS_VALID_POS_LLA_BIT);

  // struct UtmCoor_i utm_pos;      ///< position in UTM (north,east: cm; alt: mm over MSL)
  // TODO convert from LLA or ECEF ?

  // int32_t hmsl;                  ///< height above mean sea level (MSL) in mm
  state.hmsl = msg.height_msl_mm;
  SetBit(state.valid_fields, GPS_VALID_HMSL_BIT);
  
  // struct NedCoor_i ned_vel;      ///< speed NED in cm/s
  state.ned_vel.x = msg.ned_velocity[0] * 100;
  state.ned_vel.y = msg.ned_velocity[1] * 100;
  state.ned_vel.z = msg.ned_velocity[2] * 100;
  SetBit(state.valid_fields, GPS_VALID_VEL_NED_BIT);

  // uint16_t gspeed;               ///< norm of 2d ground speed in cm/s
  state.gspeed = VECT2_NORM2(state.ned_vel);
  // uint16_t speed_3d;             ///< norm of 3d speed in cm/s
  state.speed_3d = VECT3_NORM2(state.ned_vel);
  // int32_t course;                ///< GPS course over ground in rad*1e7, [0, 2*Pi]*1e7 (CW/north)
  float course = atan2f(state.ned_vel.y, state.ned_vel.x);
  NormCourseRad(course);
  state.course = course * 1e7;
  SetBit(state.valid_fields, GPS_VALID_COURSE_BIT);

  // uint32_t cacc;                 ///< course accuracy in rad*1e7
  // TODO

  if (msg.covariance.len == 6) {
    // uint32_t hacc;                 ///< horizontal accuracy in cm
    if (!isnan(msg.covariance.data[0])) {
      state.hacc = sqrtf(msg.covariance.data[0]) * 100;
    }
    // uint32_t vacc;                 ///< vertical accuracy in cm
    if (!isnan(msg.covariance.data[2])) {
      state.vacc = sqrtf(msg.covariance.data[2]) * 100;
    }
    // uint32_t pacc;                 ///< position accuracy in cm
    if(!isnan(msg.covariance.data[0]) && !isnan(msg.covariance.data[2])) {
      state.pacc = sqrtf(SQUARE(state.hacc) + SQUARE(state.vacc));
    }
    // uint32_t sacc;                 ///< speed accuracy in cm/s
    if (!isnan(msg.covariance.data[3]) &&
        !isnan(msg.covariance.data[4]) &&
        !isnan(msg.covariance.data[5])) {
      state.sacc = sqrtf((msg.covariance.data[3] + msg.covariance.data[4] + msg.covariance.data[5])/3) * 100;
    }
  } else if(msg.covariance.len != 0){
    // TODO handle the other cases ?
  }

  // uint16_t pdop;                 ///< position dilution of precision scaled by 100
  state.pdop = msg.pdop * 100;
  // uint8_t num_sv;                ///< number of sat in fix
  state.num_sv = msg.sats_used;

  // uint16_t week;                 ///< GPS week
  // uint32_t tow;                  ///< GPS time of week in ms
  const uint32_t unixToGpsEpoch = 315964800;  // Unix timestamp of the GPS epoch 1980-01-06 00:00:00 UTC
  uint64_t gnss_ts_msec = msg.gnss_timestamp.usec / 1000 - unixToGpsEpoch*1000;
  switch (msg.gnss_time_standard)
  {
  case UAVCAN_EQUIPMENT_GNSS_FIX2_GNSS_TIME_STANDARD_NONE:
    //TODO how to handle it ?
    break;
  case UAVCAN_EQUIPMENT_GNSS_FIX2_GNSS_TIME_STANDARD_TAI:
    gnss_ts_msec -= 19000;
    /* code */
    break;
  case UAVCAN_EQUIPMENT_GNSS_FIX2_GNSS_TIME_STANDARD_UTC:
    gnss_ts_msec += (msg.num_leap_seconds*1000 - 9000);
    /* code */
    break;
  case UAVCAN_EQUIPMENT_GNSS_FIX2_GNSS_TIME_STANDARD_GPS:
    // good
    break;
  default:
    break;
  }
  
  state.week = gnss_ts_msec / SECS_IN_WEEK;
  state.tow = (gnss_ts_msec % SECS_IN_WEEK);
  
  
  // TODO what are channels ? Can we get SVinfo ?
  // uint8_t nb_channels;           ///< Number of scanned satellites
  state.nb_channels = msg.sats_used;
  // struct SVinfo svinfos[GPS_NB_CHANNELS]; ///< holds information from the Space Vehicles (Satellites)


  // uint16_t reset;                ///< hotstart, warmstart, coldstart
  
  AbiSendMsgGPS(state.comp_id, now_ts, &state);
}


void gps_uavcan_init(void)
{
  // your init code here
  uavcan_bind(
    UAVCAN_EQUIPMENT_GNSS_FIX2_ID,
    UAVCAN_EQUIPMENT_GNSS_FIX2_SIGNATURE,
    &gps_uavcan_ev, &gps_uavcan_cb);
}


