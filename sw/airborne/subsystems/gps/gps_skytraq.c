/*
 * Copyright (C) 2010 Antoine Drouin <poinix@gmail.com>
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

#include "subsystems/gps.h"
#include "led.h"

#if GPS_USE_LATLONG
/* currently needed to get nav_utm_zone0 */
#include "subsystems/navigation/common_nav.h"
#include "math/pprz_geodetic_float.h"
#endif

struct GpsSkytraq gps_skytraq;

/* parser status */
#define UNINIT        0
#define GOT_SYNC1     1
#define GOT_SYNC2     2
#define GOT_LEN1      3
#define GOT_LEN2      4
#define GOT_ID        5
#define GOT_PAYLOAD   6
#define GOT_CHECKSUM  7
#define GOT_SYNC3     8

#define SKYTRAQ_FIX_NONE    0x00
#define SKYTRAQ_FIX_2D      0x01
#define SKYTRAQ_FIX_3D      0x02
#define SKYTRAQ_FIX_3D_DGPS 0x03


#define SKYTRAQ_SYNC1 0xA0
#define SKYTRAQ_SYNC2 0xA1

#define SKYTRAQ_SYNC3 0x0D
#define SKYTRAQ_SYNC4 0x0A


static inline uint16_t bswap16(uint16_t a)
{
  return (a << 8) | (a >> 8);
}

#define SKYTRAQ_NAVIGATION_DATA_FixMode(_payload) (uint8_t) (*((uint8_t*)_payload+2-2))
#define SKYTRAQ_NAVIGATION_DATA_NumSV(_payload)   (uint8_t) (*((uint8_t*)_payload+3-2))

#define SKYTRAQ_NAVIGATION_DATA_WEEK(_payload)    bswap16(*(uint16_t*)&_payload[4-2])
#define SKYTRAQ_NAVIGATION_DATA_TOW(_payload)     __builtin_bswap32(*(uint32_t*)&_payload[6-2])

#define SKYTRAQ_NAVIGATION_DATA_LAT(_payload)     (int32_t)__builtin_bswap32(*( int32_t*)&_payload[10-2])
#define SKYTRAQ_NAVIGATION_DATA_LON(_payload)     (int32_t)__builtin_bswap32(*( int32_t*)&_payload[14-2])

#define SKYTRAQ_NAVIGATION_DATA_AEL(_payload)     __builtin_bswap32(*(uint32_t*)&_payload[18-2])
#define SKYTRAQ_NAVIGATION_DATA_ASL(_payload)     __builtin_bswap32(*(uint32_t*)&_payload[22-2])

#define SKYTRAQ_NAVIGATION_DATA_GDOP(_payload)    bswap16(*(uint16_t*)&_payload[26-2])
#define SKYTRAQ_NAVIGATION_DATA_PDOP(_payload)    bswap16(*(uint16_t*)&_payload[28-2])
#define SKYTRAQ_NAVIGATION_DATA_HDOP(_payload)    bswap16(*(uint16_t*)&_payload[30-2])
#define SKYTRAQ_NAVIGATION_DATA_VDOP(_payload)    bswap16(*(uint16_t*)&_payload[32-2])
#define SKYTRAQ_NAVIGATION_DATA_TDOP(_payload)    bswap16(*(uint16_t*)&_payload[34-2])

#define SKYTRAQ_NAVIGATION_DATA_ECEFX(_payload)   (int32_t)__builtin_bswap32(*(uint32_t*)&_payload[36-2])
#define SKYTRAQ_NAVIGATION_DATA_ECEFY(_payload)   (int32_t)__builtin_bswap32(*(uint32_t*)&_payload[40-2])
#define SKYTRAQ_NAVIGATION_DATA_ECEFZ(_payload)   (int32_t)__builtin_bswap32(*(uint32_t*)&_payload[44-2])
#define SKYTRAQ_NAVIGATION_DATA_ECEFVX(_payload)  (int32_t)__builtin_bswap32(*(uint32_t*)&_payload[48-2])
#define SKYTRAQ_NAVIGATION_DATA_ECEFVY(_payload)  (int32_t)__builtin_bswap32(*(uint32_t*)&_payload[52-2])
#define SKYTRAQ_NAVIGATION_DATA_ECEFVZ(_payload)  (int32_t)__builtin_bswap32(*(uint32_t*)&_payload[56-2])


// distance in cm (10km dist max any direction)
#define MAX_DISTANCE  1000000

static int distance_too_great(struct EcefCoor_i *ecef_ref, struct EcefCoor_i *ecef_pos);

void gps_impl_init(void)
{

  gps_skytraq.status = UNINIT;

}

void gps_skytraq_msg(void (* _cb)(void))
{
  gps.last_msg_ticks = sys_time.nb_sec_rem;
  gps.last_msg_time = sys_time.nb_sec;
  gps_skytraq_read_message();
  if (gps_skytraq.msg_id == SKYTRAQ_ID_NAVIGATION_DATA) {
    if (gps.fix == GPS_FIX_3D) {
      gps.last_3dfix_ticks = sys_time.nb_sec_rem;
      gps.last_3dfix_time = sys_time.nb_sec;
    }
    _cb();
  }
  gps_skytraq.msg_available = FALSE;
}


void gps_skytraq_read_message(void)
{

  if (gps_skytraq.msg_id == SKYTRAQ_ID_NAVIGATION_DATA) {
    gps.ecef_pos.x  = SKYTRAQ_NAVIGATION_DATA_ECEFX(gps_skytraq.msg_buf);
    gps.ecef_pos.y  = SKYTRAQ_NAVIGATION_DATA_ECEFY(gps_skytraq.msg_buf);
    gps.ecef_pos.z  = SKYTRAQ_NAVIGATION_DATA_ECEFZ(gps_skytraq.msg_buf);
    gps.ecef_vel.x  = SKYTRAQ_NAVIGATION_DATA_ECEFVX(gps_skytraq.msg_buf);
    gps.ecef_vel.y  = SKYTRAQ_NAVIGATION_DATA_ECEFVY(gps_skytraq.msg_buf);
    gps.ecef_vel.z  = SKYTRAQ_NAVIGATION_DATA_ECEFVZ(gps_skytraq.msg_buf);
    gps.lla_pos.lat = SKYTRAQ_NAVIGATION_DATA_LAT(gps_skytraq.msg_buf);
    gps.lla_pos.lon = SKYTRAQ_NAVIGATION_DATA_LON(gps_skytraq.msg_buf);
    gps.lla_pos.alt = SKYTRAQ_NAVIGATION_DATA_AEL(gps_skytraq.msg_buf) * 10;
    gps.hmsl        = SKYTRAQ_NAVIGATION_DATA_ASL(gps_skytraq.msg_buf) * 10;
    //   pacc;
    //   sacc;
    gps.pdop        = SKYTRAQ_NAVIGATION_DATA_PDOP(gps_skytraq.msg_buf);
    gps.num_sv      = SKYTRAQ_NAVIGATION_DATA_NumSV(gps_skytraq.msg_buf);
    gps.tow         = SKYTRAQ_NAVIGATION_DATA_TOW(gps_skytraq.msg_buf) * 10;

    switch (SKYTRAQ_NAVIGATION_DATA_FixMode(gps_skytraq.msg_buf)) {
      case SKYTRAQ_FIX_3D_DGPS:
      case SKYTRAQ_FIX_3D:
        gps.fix = GPS_FIX_3D;
        break;
      case SKYTRAQ_FIX_2D:
        gps.fix = GPS_FIX_2D;
        break;
      default:
        gps.fix = GPS_FIX_NONE;
    }

#if GPS_USE_LATLONG
    /* Computes from (lat, long) in the referenced UTM zone */
    struct LlaCoor_f lla_f;
    LLA_FLOAT_OF_BFP(lla_f, gps.lla_pos);
    struct UtmCoor_f utm_f;
    utm_f.zone = nav_utm_zone0;
    /* convert to utm */
    utm_of_lla_f(&utm_f, &lla_f);
    /* copy results of utm conversion */
    gps.utm_pos.east = utm_f.east * 100;
    gps.utm_pos.north = utm_f.north * 100;
    gps.utm_pos.alt = gps.lla_pos.alt;
    gps.utm_pos.zone = nav_utm_zone0;
#endif

    if (gps.fix == GPS_FIX_3D) {
      if (distance_too_great(&gps_skytraq.ref_ltp.ecef, &gps.ecef_pos)) {
        // just grab current ecef_pos as reference.
        ltp_def_from_ecef_i(&gps_skytraq.ref_ltp, &gps.ecef_pos);
      }
      // convert ecef velocity vector to NED vector.
      ned_of_ecef_vect_i(&gps.ned_vel, &gps_skytraq.ref_ltp, &gps.ecef_vel);

      // ground course in radians
      gps.course = (atan2f((float)gps.ned_vel.y, (float)gps.ned_vel.x)) * 1e7;
      // GT: gps.cacc = ... ? what should course accuracy be?

      // ground speed
      gps.gspeed = sqrt(gps.ned_vel.x * gps.ned_vel.x + gps.ned_vel.y * gps.ned_vel.y);
      gps.speed_3d = sqrt(gps.ned_vel.x * gps.ned_vel.x + gps.ned_vel.y * gps.ned_vel.y + gps.ned_vel.z * gps.ned_vel.z);

      // vertical speed (climb)
      // solved by gps.ned.z?
    }


#ifdef GPS_LED
    if (gps.fix == GPS_FIX_3D) {
      LED_ON(GPS_LED);
    } else {
      LED_TOGGLE(GPS_LED);
    }
#endif
  }

}

void gps_skytraq_parse(uint8_t c)
{
  if (gps_skytraq.status < GOT_PAYLOAD) {
    gps_skytraq.checksum ^= c;
  }
  switch (gps_skytraq.status) {
    case UNINIT:
      if (c == SKYTRAQ_SYNC1) {
        gps_skytraq.status = GOT_SYNC1;
      }
      break;
    case GOT_SYNC1:
      if (c != SKYTRAQ_SYNC2) {
        gps_skytraq.error_last = GPS_SKYTRAQ_ERR_OUT_OF_SYNC;
        goto error;
      }
      gps_skytraq.status = GOT_SYNC2;
      break;
    case GOT_SYNC2:
      gps_skytraq.len = c << 8;
      gps_skytraq.status = GOT_LEN1;
      break;
    case GOT_LEN1:
      gps_skytraq.len += c;
      gps_skytraq.status = GOT_LEN2;
      if (gps_skytraq.len > GPS_SKYTRAQ_MAX_PAYLOAD) {
        gps_skytraq.error_last = GPS_SKYTRAQ_ERR_MSG_TOO_LONG;
        goto error;
      }
      break;
    case GOT_LEN2:
      gps_skytraq.msg_id = c;
      gps_skytraq.msg_idx = 0;
      gps_skytraq.checksum = c;
      gps_skytraq.status = GOT_ID;
      break;
    case GOT_ID:
      gps_skytraq.msg_buf[gps_skytraq.msg_idx] = c;
      gps_skytraq.msg_idx++;
      if (gps_skytraq.msg_idx >= gps_skytraq.len - 1) {
        gps_skytraq.status = GOT_PAYLOAD;
      }
      break;
    case GOT_PAYLOAD:
      if (c != gps_skytraq.checksum) {
        gps_skytraq.error_last = GPS_SKYTRAQ_ERR_CHECKSUM;
        goto error;
      }
      gps_skytraq.status = GOT_CHECKSUM;
      break;
    case GOT_CHECKSUM:
      if (c != SKYTRAQ_SYNC3) {
        gps_skytraq.error_last = GPS_SKYTRAQ_ERR_OUT_OF_SYNC;
        goto error;
      }
      gps_skytraq.status = GOT_SYNC3;
      break;
    case GOT_SYNC3:
      gps_skytraq.msg_available = TRUE;
      goto restart;
    default:
      gps_skytraq.error_last = GPS_SKYTRAQ_ERR_UNEXPECTED;
      goto error;
  }
  return;
error:
  gps_skytraq.error_cnt++;
restart:
  gps_skytraq.status = UNINIT;
  return;
}

static int distance_too_great(struct EcefCoor_i *ecef_ref, struct EcefCoor_i *ecef_pos)
{
  int32_t xdiff = abs(ecef_ref->x - ecef_pos->x);
  if (xdiff > MAX_DISTANCE) {
    return TRUE;
  }
  int32_t ydiff = abs(ecef_ref->y - ecef_pos->y);
  if (ydiff > MAX_DISTANCE) {
    return TRUE;
  }
  int32_t zdiff = abs(ecef_ref->z - ecef_pos->z);
  if (zdiff > MAX_DISTANCE) {
    return TRUE;
  }

  return FALSE;
}
