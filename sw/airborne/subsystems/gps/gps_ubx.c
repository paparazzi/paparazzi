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
#include "subsystems/abi.h"
#include "led.h"

#if GPS_USE_LATLONG
/* currently needed to get nav_utm_zone0 */
#include "subsystems/navigation/common_nav.h"
#include "math/pprz_geodetic_float.h"
#endif

/** Includes macros generated from ubx.xml */
#include "ubx_protocol.h"

/* parser status */
#define UNINIT        0
#define GOT_SYNC1     1
#define GOT_SYNC2     2
#define GOT_CLASS     3
#define GOT_ID        4
#define GOT_LEN1      5
#define GOT_LEN2      6
#define GOT_PAYLOAD   7
#define GOT_CHECKSUM1 8

/* last error type */
#define GPS_UBX_ERR_NONE         0
#define GPS_UBX_ERR_OVERRUN      1
#define GPS_UBX_ERR_MSG_TOO_LONG 2
#define GPS_UBX_ERR_CHECKSUM     3
#define GPS_UBX_ERR_UNEXPECTED   4
#define GPS_UBX_ERR_OUT_OF_SYNC  5

#define UTM_HEM_NORTH 0
#define UTM_HEM_SOUTH 1

struct GpsUbx gps_ubx;

#if USE_GPS_UBX_RXM_RAW
struct GpsUbxRaw gps_ubx_raw;
#endif

void gps_impl_init(void)
{
  gps_ubx.status = UNINIT;
  gps_ubx.msg_available = FALSE;
  gps_ubx.error_cnt = 0;
  gps_ubx.error_last = GPS_UBX_ERR_NONE;
  gps_ubx.have_velned = 0;
}


void gps_ubx_read_message(void)
{

  if (gps_ubx.msg_class == UBX_NAV_ID) {
    if (gps_ubx.msg_id == UBX_NAV_SOL_ID) {
      /* get hardware clock ticks */
      gps_time_sync.t0_ticks      = sys_time.nb_tick;
      gps_time_sync.t0_tow        = UBX_NAV_SOL_ITOW(gps_ubx.msg_buf);
      gps_time_sync.t0_tow_frac   = UBX_NAV_SOL_Frac(gps_ubx.msg_buf);
      gps.tow        = UBX_NAV_SOL_ITOW(gps_ubx.msg_buf);
      gps.week       = UBX_NAV_SOL_week(gps_ubx.msg_buf);
      gps.fix        = UBX_NAV_SOL_GPSfix(gps_ubx.msg_buf);
      gps.ecef_pos.x = UBX_NAV_SOL_ECEF_X(gps_ubx.msg_buf);
      gps.ecef_pos.y = UBX_NAV_SOL_ECEF_Y(gps_ubx.msg_buf);
      gps.ecef_pos.z = UBX_NAV_SOL_ECEF_Z(gps_ubx.msg_buf);
      gps.pacc       = UBX_NAV_SOL_Pacc(gps_ubx.msg_buf);
      gps.ecef_vel.x = UBX_NAV_SOL_ECEFVX(gps_ubx.msg_buf);
      gps.ecef_vel.y = UBX_NAV_SOL_ECEFVY(gps_ubx.msg_buf);
      gps.ecef_vel.z = UBX_NAV_SOL_ECEFVZ(gps_ubx.msg_buf);
      gps.sacc       = UBX_NAV_SOL_Sacc(gps_ubx.msg_buf);
      gps.pdop       = UBX_NAV_SOL_PDOP(gps_ubx.msg_buf);
      gps.num_sv     = UBX_NAV_SOL_numSV(gps_ubx.msg_buf);
#ifdef GPS_LED
      if (gps.fix == GPS_FIX_3D) {
        LED_ON(GPS_LED);
      } else {
        LED_TOGGLE(GPS_LED);
      }
#endif
    } else if (gps_ubx.msg_id == UBX_NAV_POSLLH_ID) {
      gps.lla_pos.lat = UBX_NAV_POSLLH_LAT(gps_ubx.msg_buf);
      gps.lla_pos.lon = UBX_NAV_POSLLH_LON(gps_ubx.msg_buf);
      gps.lla_pos.alt = UBX_NAV_POSLLH_HEIGHT(gps_ubx.msg_buf);
      gps.hmsl        = UBX_NAV_POSLLH_HMSL(gps_ubx.msg_buf);
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
#else
    } else if (gps_ubx.msg_id == UBX_NAV_POSUTM_ID) {
      gps.utm_pos.east = UBX_NAV_POSUTM_EAST(gps_ubx.msg_buf);
      gps.utm_pos.north = UBX_NAV_POSUTM_NORTH(gps_ubx.msg_buf);
      uint8_t hem = UBX_NAV_POSUTM_HEM(gps_ubx.msg_buf);
      if (hem == UTM_HEM_SOUTH) {
        gps.utm_pos.north -= 1000000000;  /* Subtract false northing: -10000km */
      }
      gps.utm_pos.alt = UBX_NAV_POSUTM_ALT(gps_ubx.msg_buf) * 10;
      gps.hmsl = gps.utm_pos.alt;
      gps.lla_pos.alt = gps.utm_pos.alt; // FIXME: with UTM only you do not receive ellipsoid altitude
      gps.utm_pos.zone = UBX_NAV_POSUTM_ZONE(gps_ubx.msg_buf);
#endif
    } else if (gps_ubx.msg_id == UBX_NAV_VELNED_ID) {
      gps.speed_3d = UBX_NAV_VELNED_Speed(gps_ubx.msg_buf);
      gps.gspeed = UBX_NAV_VELNED_GSpeed(gps_ubx.msg_buf);
      gps.ned_vel.x = UBX_NAV_VELNED_VEL_N(gps_ubx.msg_buf);
      gps.ned_vel.y = UBX_NAV_VELNED_VEL_E(gps_ubx.msg_buf);
      gps.ned_vel.z = UBX_NAV_VELNED_VEL_D(gps_ubx.msg_buf);
      // Ublox gives I4 heading in 1e-5 degrees, apparenty from 0 to 360 degrees (not -180 to 180)
      // I4 max = 2^31 = 214 * 1e5 * 100 < 360 * 1e7: overflow on angles over 214 deg -> casted to -214 deg
      // solution: First to radians, and then scale to 1e-7 radians
      // First x 10 for loosing less resolution, then to radians, then multiply x 10 again
      gps.course = (RadOfDeg(UBX_NAV_VELNED_Heading(gps_ubx.msg_buf) * 10)) * 10;
      gps.cacc = (RadOfDeg(UBX_NAV_VELNED_CAcc(gps_ubx.msg_buf) * 10)) * 10;
      gps.tow = UBX_NAV_VELNED_ITOW(gps_ubx.msg_buf);
      gps_ubx.have_velned = 1;
    } else if (gps_ubx.msg_id == UBX_NAV_SVINFO_ID) {
      gps.nb_channels = Min(UBX_NAV_SVINFO_NCH(gps_ubx.msg_buf), GPS_NB_CHANNELS);
      uint8_t i;
      for (i = 0; i < gps.nb_channels; i++) {
        gps.svinfos[i].svid = UBX_NAV_SVINFO_SVID(gps_ubx.msg_buf, i);
        gps.svinfos[i].flags = UBX_NAV_SVINFO_Flags(gps_ubx.msg_buf, i);
        gps.svinfos[i].qi = UBX_NAV_SVINFO_QI(gps_ubx.msg_buf, i);
        gps.svinfos[i].cno = UBX_NAV_SVINFO_CNO(gps_ubx.msg_buf, i);
        gps.svinfos[i].elev = UBX_NAV_SVINFO_Elev(gps_ubx.msg_buf, i);
        gps.svinfos[i].azim = UBX_NAV_SVINFO_Azim(gps_ubx.msg_buf, i);
      }
    } else if (gps_ubx.msg_id == UBX_NAV_STATUS_ID) {
      gps.fix = UBX_NAV_STATUS_GPSfix(gps_ubx.msg_buf);
      gps_ubx.status_flags = UBX_NAV_STATUS_Flags(gps_ubx.msg_buf);
      gps_ubx.sol_flags = UBX_NAV_SOL_Flags(gps_ubx.msg_buf);
    }
  }
#if USE_GPS_UBX_RXM_RAW
  else if (gps_ubx.msg_class == UBX_RXM_ID) {
    if (gps_ubx.msg_id == UBX_RXM_RAW_ID) {
      gps_ubx_raw.iTOW = UBX_RXM_RAW_iTOW(gps_ubx.msg_buf);
      gps_ubx_raw.week = UBX_RXM_RAW_week(gps_ubx.msg_buf);
      gps_ubx_raw.numSV = UBX_RXM_RAW_numSV(gps_ubx.msg_buf);
      uint8_t i;
      for (i = 0; i < gps_ubx_raw.numSV; i++) {
        gps_ubx_raw.measures[i].cpMes = UBX_RXM_RAW_cpMes(gps_ubx.msg_buf, i);
        gps_ubx_raw.measures[i].prMes = UBX_RXM_RAW_prMes(gps_ubx.msg_buf, i);
        gps_ubx_raw.measures[i].doMes = UBX_RXM_RAW_doMes(gps_ubx.msg_buf, i);
        gps_ubx_raw.measures[i].sv = UBX_RXM_RAW_sv(gps_ubx.msg_buf, i);
        gps_ubx_raw.measures[i].mesQI = UBX_RXM_RAW_mesQI(gps_ubx.msg_buf, i);
        gps_ubx_raw.measures[i].cno = UBX_RXM_RAW_cno(gps_ubx.msg_buf, i);
        gps_ubx_raw.measures[i].lli = UBX_RXM_RAW_lli(gps_ubx.msg_buf, i);
      }
    }
  }
#endif
}

#if LOG_RAW_GPS
#include "sdLog.h"
#include "subsystems/chibios-libopencm3/chibios_sdlog.h"
#endif

/* UBX parsing */
void gps_ubx_parse(uint8_t c)
{
#if LOG_RAW_GPS
  sdLogWriteByte(pprzLogFile, c);
#endif
  if (gps_ubx.status < GOT_PAYLOAD) {
    gps_ubx.ck_a += c;
    gps_ubx.ck_b += gps_ubx.ck_a;
  }
  switch (gps_ubx.status) {
    case UNINIT:
      if (c == UBX_SYNC1) {
        gps_ubx.status++;
      }
      break;
    case GOT_SYNC1:
      if (c != UBX_SYNC2) {
        gps_ubx.error_last = GPS_UBX_ERR_OUT_OF_SYNC;
        goto error;
      }
      gps_ubx.ck_a = 0;
      gps_ubx.ck_b = 0;
      gps_ubx.status++;
      break;
    case GOT_SYNC2:
      if (gps_ubx.msg_available) {
        /* Previous message has not yet been parsed: discard this one */
        gps_ubx.error_last = GPS_UBX_ERR_OVERRUN;
        goto error;
      }
      gps_ubx.msg_class = c;
      gps_ubx.status++;
      break;
    case GOT_CLASS:
      gps_ubx.msg_id = c;
      gps_ubx.status++;
      break;
    case GOT_ID:
      gps_ubx.len = c;
      gps_ubx.status++;
      break;
    case GOT_LEN1:
      gps_ubx.len |= (c << 8);
      if (gps_ubx.len > GPS_UBX_MAX_PAYLOAD) {
        gps_ubx.error_last = GPS_UBX_ERR_MSG_TOO_LONG;
        goto error;
      }
      gps_ubx.msg_idx = 0;
      gps_ubx.status++;
      break;
    case GOT_LEN2:
      gps_ubx.msg_buf[gps_ubx.msg_idx] = c;
      gps_ubx.msg_idx++;
      if (gps_ubx.msg_idx >= gps_ubx.len) {
        gps_ubx.status++;
      }
      break;
    case GOT_PAYLOAD:
      if (c != gps_ubx.ck_a) {
        gps_ubx.error_last = GPS_UBX_ERR_CHECKSUM;
        goto error;
      }
      gps_ubx.status++;
      break;
    case GOT_CHECKSUM1:
      if (c != gps_ubx.ck_b) {
        gps_ubx.error_last = GPS_UBX_ERR_CHECKSUM;
        goto error;
      }
      gps_ubx.msg_available = TRUE;
      goto restart;
      break;
    default:
      gps_ubx.error_last = GPS_UBX_ERR_UNEXPECTED;
      goto error;
  }
  return;
error:
  gps_ubx.error_cnt++;
restart:
  gps_ubx.status = UNINIT;
  return;
}

static void ubx_send_1byte(struct link_device *dev, uint8_t byte)
{
  dev->put_byte(dev->periph, byte);
  gps_ubx.send_ck_a += byte;
  gps_ubx.send_ck_b += gps_ubx.send_ck_a;
}

void ubx_header(struct link_device *dev, uint8_t nav_id, uint8_t msg_id, uint16_t len)
{
  dev->put_byte(dev->periph, UBX_SYNC1);
  dev->put_byte(dev->periph, UBX_SYNC2);
  gps_ubx.send_ck_a = 0;
  gps_ubx.send_ck_b = 0;
  ubx_send_1byte(dev, nav_id);
  ubx_send_1byte(dev, msg_id);
  ubx_send_1byte(dev, (uint8_t)(len&0xFF));
  ubx_send_1byte(dev, (uint8_t)(len>>8));
}

void ubx_trailer(struct link_device *dev)
{
  dev->put_byte(dev->periph, gps_ubx.send_ck_a);
  dev->put_byte(dev->periph, gps_ubx.send_ck_b);
  dev->send_message(dev->periph);
}

void ubx_send_bytes(struct link_device *dev, uint8_t len, uint8_t *bytes)
{
  int i;
  for (i = 0; i < len; i++) {
    ubx_send_1byte(dev, bytes[i]);
  }
}

void ubx_send_cfg_rst(struct link_device *dev, uint16_t bbr , uint8_t reset_mode)
{
#ifdef GPS_LINK
  UbxSend_CFG_RST(dev, bbr, reset_mode, 0x00);
#endif /* else less harmful for HITL */
}

#ifndef GPS_UBX_UCENTER
#define gps_ubx_ucenter_event() {}
#else
#include "modules/gps/gps_ubx_ucenter.h"
#endif

void gps_ubx_msg(void)
{
  // current timestamp
  uint32_t now_ts = get_sys_time_usec();

  gps.last_msg_ticks = sys_time.nb_sec_rem;
  gps.last_msg_time = sys_time.nb_sec;
  gps_ubx_read_message();
  gps_ubx_ucenter_event();
  if (gps_ubx.msg_class == UBX_NAV_ID &&
      (gps_ubx.msg_id == UBX_NAV_VELNED_ID ||
       (gps_ubx.msg_id == UBX_NAV_SOL_ID &&
        gps_ubx.have_velned == 0))) {
    if (gps.fix == GPS_FIX_3D) {
      gps.last_3dfix_ticks = sys_time.nb_sec_rem;
      gps.last_3dfix_time = sys_time.nb_sec;
    }
    AbiSendMsgGPS(GPS_UBX_ID, now_ts, &gps);
  }
  gps_ubx.msg_available = FALSE;
}

