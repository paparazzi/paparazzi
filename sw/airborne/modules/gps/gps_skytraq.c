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

#include "modules/gps/gps.h"
#include "modules/gps/gps_skytraq.h"
#include "modules/core/abi.h"
#include "led.h"
#include "pprzlink/pprzlink_device.h"

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

struct GpsSkytraq gps_skytraq;

void gps_skytraq_read_message(void);
void gps_skytraq_parse(uint8_t c);
void gps_skytraq_msg(void);

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

void gps_skytraq_init(void)
{
  gps_skytraq.status = UNINIT;
}

void gps_skytraq_msg(void)
{
  // current timestamp
  uint32_t now_ts = get_sys_time_usec();
  gps_skytraq.state.last_msg_ticks = sys_time.nb_sec_rem;
  gps_skytraq.state.last_msg_time = sys_time.nb_sec;
  gps_skytraq_read_message();
  if (gps_skytraq.msg_id == SKYTRAQ_ID_NAVIGATION_DATA) {
    if (gps_skytraq.state.fix == GPS_FIX_3D) {
      gps_skytraq.state.last_3dfix_ticks = sys_time.nb_sec_rem;
      gps_skytraq.state.last_3dfix_time = sys_time.nb_sec;
    }
    AbiSendMsgGPS(GPS_SKYTRAQ_ID, now_ts, &gps_skytraq.state);
  }
  gps_skytraq.msg_available = false;
}

void gps_skytraq_event(void)
{
  struct link_device *dev = &((SKYTRAQ_GPS_LINK).device);

  while (dev->char_available(dev->periph)) {
    gps_skytraq_parse(dev->get_byte(dev->periph));
    if (gps_skytraq.msg_available) {
      gps_skytraq_msg();
    }
  }
}

void gps_skytraq_read_message(void)
{

  if (gps_skytraq.msg_id == SKYTRAQ_ID_NAVIGATION_DATA) {
    gps_skytraq.state.ecef_pos.x  = SKYTRAQ_NAVIGATION_DATA_ECEFX(gps_skytraq.msg_buf);
    gps_skytraq.state.ecef_pos.y  = SKYTRAQ_NAVIGATION_DATA_ECEFY(gps_skytraq.msg_buf);
    gps_skytraq.state.ecef_pos.z  = SKYTRAQ_NAVIGATION_DATA_ECEFZ(gps_skytraq.msg_buf);
    SetBit(gps_skytraq.state.valid_fields, GPS_VALID_POS_ECEF_BIT);

    gps_skytraq.state.ecef_vel.x  = SKYTRAQ_NAVIGATION_DATA_ECEFVX(gps_skytraq.msg_buf);
    gps_skytraq.state.ecef_vel.y  = SKYTRAQ_NAVIGATION_DATA_ECEFVY(gps_skytraq.msg_buf);
    gps_skytraq.state.ecef_vel.z  = SKYTRAQ_NAVIGATION_DATA_ECEFVZ(gps_skytraq.msg_buf);
    SetBit(gps_skytraq.state.valid_fields, GPS_VALID_VEL_ECEF_BIT);

    gps_skytraq.state.lla_pos.lat = SKYTRAQ_NAVIGATION_DATA_LAT(gps_skytraq.msg_buf);
    gps_skytraq.state.lla_pos.lon = SKYTRAQ_NAVIGATION_DATA_LON(gps_skytraq.msg_buf);
    gps_skytraq.state.lla_pos.alt = SKYTRAQ_NAVIGATION_DATA_AEL(gps_skytraq.msg_buf) * 10;
    SetBit(gps_skytraq.state.valid_fields, GPS_VALID_POS_LLA_BIT);

    gps_skytraq.state.hmsl        = SKYTRAQ_NAVIGATION_DATA_ASL(gps_skytraq.msg_buf) * 10;
    SetBit(gps_skytraq.state.valid_fields, GPS_VALID_HMSL_BIT);

    //   pacc;
    //   sacc;
    gps_skytraq.state.pdop        = SKYTRAQ_NAVIGATION_DATA_PDOP(gps_skytraq.msg_buf);
    gps_skytraq.state.num_sv      = SKYTRAQ_NAVIGATION_DATA_NumSV(gps_skytraq.msg_buf);
    gps_skytraq.state.tow         = SKYTRAQ_NAVIGATION_DATA_TOW(gps_skytraq.msg_buf) * 10;

    switch (SKYTRAQ_NAVIGATION_DATA_FixMode(gps_skytraq.msg_buf)) {
      case SKYTRAQ_FIX_3D_DGPS:
      case SKYTRAQ_FIX_3D:
        gps_skytraq.state.fix = GPS_FIX_3D;
        break;
      case SKYTRAQ_FIX_2D:
        gps_skytraq.state.fix = GPS_FIX_2D;
        break;
      default:
        gps_skytraq.state.fix = GPS_FIX_NONE;
    }

    if (gps_skytraq.state.fix == GPS_FIX_3D) {
      if (distance_too_great(&gps_skytraq.ref_ltp.ecef, &gps_skytraq.state.ecef_pos)) {
        // just grab current ecef_pos as reference.
        ltp_def_from_ecef_i(&gps_skytraq.ref_ltp, &gps_skytraq.state.ecef_pos);
      }
      // convert ecef velocity vector to NED vector.
      ned_of_ecef_vect_i(&gps_skytraq.state.ned_vel, &gps_skytraq.ref_ltp, &gps_skytraq.state.ecef_vel);
      SetBit(gps_skytraq.state.valid_fields, GPS_VALID_VEL_NED_BIT);

      // ground course in radians
      gps_skytraq.state.course = (atan2f((float)gps_skytraq.state.ned_vel.y, (float)gps_skytraq.state.ned_vel.x)) * 1e7;
      SetBit(gps_skytraq.state.valid_fields, GPS_VALID_COURSE_BIT);
      // GT: gps_skytraq.state.cacc = ... ? what should course accuracy be?

      // ground speed
      gps_skytraq.state.gspeed = sqrt(gps_skytraq.state.ned_vel.x * gps_skytraq.state.ned_vel.x + gps_skytraq.state.ned_vel.y * gps_skytraq.state.ned_vel.y);
      gps_skytraq.state.speed_3d = sqrt(gps_skytraq.state.ned_vel.x * gps_skytraq.state.ned_vel.x + gps_skytraq.state.ned_vel.y * gps_skytraq.state.ned_vel.y + gps_skytraq.state.ned_vel.z * gps_skytraq.state.ned_vel.z);

      // vertical speed (climb)
      // solved by gps_skytraq.state.ned.z?
    }


#ifdef GPS_LED
    if (gps_skytraq.state.fix == GPS_FIX_3D) {
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
      gps_skytraq.msg_available = true;
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
    return true;
  }
  int32_t ydiff = abs(ecef_ref->y - ecef_pos->y);
  if (ydiff > MAX_DISTANCE) {
    return true;
  }
  int32_t zdiff = abs(ecef_ref->z - ecef_pos->z);
  if (zdiff > MAX_DISTANCE) {
    return true;
  }

  return false;
}
