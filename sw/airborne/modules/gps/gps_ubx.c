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


#include "modules/gps/gps_ubx.h"
#include "modules/core/abi.h"
#include "pprzlink/dl_protocol.h"
#include "led.h"

#ifndef USE_GPS_UBX_RTCM
#define USE_GPS_UBX_RTCM 0
#endif


#if USE_GPS_UBX_RTCM
#include "modules/gps/librtcm3/CRC24Q.h"
#define RTCM3_PREAMBLE 0xD3
#define RTCM3_MSG_1005 0x69
#define RTCM3_MSG_1077 0xB1
#define RTCM3_MSG_4072 0x72
#define RTCM3_MSG_1230 0xE6
#define RTCM3_MSG_1087 0xBB
#endif

#if PRINT_DEBUG_GPS_UBX
#define DEBUG_PRINT(...) printf(__VA_ARGS__)
#else
#define DEBUG_PRINT(...) {}
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

#define RXM_RTCM_VERSION        0x02
#define NAV_RELPOSNED_VERSION   0x01
/* last error type */
#define GPS_UBX_ERR_NONE         0
#define GPS_UBX_ERR_OVERRUN      1
#define GPS_UBX_ERR_MSG_TOO_LONG 2
#define GPS_UBX_ERR_CHECKSUM     3
#define GPS_UBX_ERR_UNEXPECTED   4
#define GPS_UBX_ERR_OUT_OF_SYNC  5

#define UTM_HEM_NORTH 0
#define UTM_HEM_SOUTH 1

struct GpsUbx gps_ubx[GPS_UBX_NB];
uint8_t gps_ubx_reset = 0;
void gps_ubx_parse(struct GpsUbx *gubx, uint8_t c);
void gps_ubx_msg(struct GpsUbx *gubx);

#if USE_GPS_UBX_RTCM
extern struct GpsRelposNED gps_relposned;
extern struct RtcmMan rtcm_man;

#ifndef INJECT_BUFF_SIZE
#define INJECT_BUFF_SIZE 1024 + 6
#endif

/* RTCM control struct type */
struct rtcm_t {
  uint32_t nbyte;                     ///< number of bytes in message buffer
  uint32_t len;                       ///< message length (bytes)
  uint8_t buff[INJECT_BUFF_SIZE];     ///< message buffer
};
struct rtcm_t rtcm = { 0 };

#endif

/*
 * GPS Reset
 */
#ifndef GPS_UBX_BOOTRESET
#define GPS_UBX_BOOTRESET 0
#endif

#define CFG_RST_Reset_Hardware 0x00
#define CFG_RST_Reset_Controlled 0x01
#define CFG_RST_Reset_Controlled_GPS_only 0x02
#define CFG_RST_Reset_Controlled_GPS_stop 0x08
#define CFG_RST_Reset_Controlled_GPS_start 0x09

#define CFG_RST_BBR_Hotstart  0x0000
#define CFG_RST_BBR_Warmstart 0x0001
#define CFG_RST_BBR_Coldstart 0xffff

void gps_ubx_init(void)
{
  gps_ubx_reset = GPS_UBX_BOOTRESET;

  // Configure the devices
  gps_ubx[0].state.comp_id = GPS_UBX_ID;
  gps_ubx[0].dev = &(UBX_GPS_PORT).device;
#if GPS_UBX_NB > 1
  gps_ubx[1].state.comp_id = GPS_UBX2_ID;
  gps_ubx[1].dev = &(UBX2_GPS_PORT).device;
#endif

  // Set the defaults
  for(uint8_t i = 0; i < GPS_UBX_NB; i++) {
    gps_ubx[i].status = UNINIT;
    gps_ubx[i].msg_available = false;
    gps_ubx[i].error_cnt = 0;
    gps_ubx[i].error_last = GPS_UBX_ERR_NONE;
    gps_ubx[i].pacc_valid = false;
  }
}

void gps_ubx_event(void)
{
  for(uint8_t i = 0; i < GPS_UBX_NB; i++) {
    struct link_device *dev = gps_ubx[i].dev;

    // Read all available bytes from the device
    while (dev->char_available(dev->periph)) {
      gps_ubx_parse(&gps_ubx[i], dev->get_byte(dev->periph));
      if (gps_ubx[i].msg_available) {
        gps_ubx_msg(&gps_ubx[i]);
      }
    }

    // Reset the GPS's if needed
    if (gps_ubx_reset > 0) {    
      switch (gps_ubx_reset) {
        case 1 : ubx_send_cfg_rst(dev, CFG_RST_BBR_Hotstart, CFG_RST_Reset_Controlled); break;
        case 2 : ubx_send_cfg_rst(dev, CFG_RST_BBR_Warmstart, CFG_RST_Reset_Controlled); break;
        case 3 : ubx_send_cfg_rst(dev, CFG_RST_BBR_Coldstart, CFG_RST_Reset_Controlled); break;
        default: DEBUG_PRINT("Unknown reset id: %i", gps_ubx[i].reset); break;
      }
    }
  }

  gps_ubx_reset = 0;
}

void gps_ubx_parse_HITL_UBX(uint8_t *buf)
{
  /** This code simulates gps_ubx.c:parse_ubx() */
  if (gps_ubx[0].msg_available) {
    gps_ubx[0].error_cnt++;
    gps_ubx[0].error_last = GPS_UBX_ERR_OVERRUN;
  } else {
    gps_ubx[0].msg_class = DL_HITL_UBX_class(buf);
    gps_ubx[0].msg_id = DL_HITL_UBX_id(buf);
    uint8_t l = DL_HITL_UBX_ubx_payload_length(buf);
    uint8_t *ubx_payload = DL_HITL_UBX_ubx_payload(buf);
    memcpy(gps_ubx[0].msg_buf, ubx_payload, l);
    gps_ubx[0].msg_available = true;
  }
}

static void gps_ubx_parse_nav_pvt(struct GpsUbx *gubx)
{
  uint8_t flags             = UBX_NAV_PVT_flags(gubx->msg_buf);

  // Copy fix information
  uint8_t gnssFixOK         = bit_is_set(flags, 0);
  uint8_t diffSoln          = bit_is_set(flags, 1);
  uint8_t carrSoln          = (flags & 0xC0) >> 6;
  if (diffSoln && carrSoln == 2) {
    gubx->state.fix = 5; // rtk
  } else if(diffSoln && carrSoln == 1) {
    gubx->state.fix = 4; // dgnss
  } else if(gnssFixOK) {
    gubx->state.fix = 3; // 3D
  } else {
    gubx->state.fix = 0;
  }

  // Copy time information
  gubx->state.tow         = UBX_NAV_PVT_iTOW(gubx->msg_buf);
  uint16_t year             = UBX_NAV_PVT_year(gubx->msg_buf);
  uint8_t month             = UBX_NAV_PVT_month(gubx->msg_buf);
  uint8_t day               = UBX_NAV_PVT_day(gubx->msg_buf);
  gubx->state.week        = gps_week_number(year, month, day);
  gubx->state.num_sv      = UBX_NAV_PVT_numSV(gubx->msg_buf);

  // Copy LLA position
  gubx->state.lla_pos.lat = UBX_NAV_PVT_lat(gubx->msg_buf);
  gubx->state.lla_pos.lon = UBX_NAV_PVT_lon(gubx->msg_buf);
  gubx->state.lla_pos.alt = UBX_NAV_PVT_height(gubx->msg_buf);
  SetBit(gubx->state.valid_fields, GPS_VALID_POS_LLA_BIT);

  // Ublox gives I4 heading in 1e-5 degrees, apparenty from 0 to 360 degrees (not -180 to 180)
  // I4 max = 2^31 = 214 * 1e5 * 100 < 360 * 1e7: overflow on angles over 214 deg -> casted to -214 deg
  // solution: First to radians, and then scale to 1e-7 radians
  // First x 10 for loosing less resolution, then to radians, then multiply x 10 again
  gubx->state.course    = (RadOfDeg(UBX_NAV_PVT_headMot(gubx->msg_buf) * 10)) * 10;
  SetBit(gubx->state.valid_fields, GPS_VALID_COURSE_BIT);
  gubx->state.cacc      = (RadOfDeg(UBX_NAV_PVT_headAcc(gubx->msg_buf) * 10)) * 10;

  // Copy HMSL and ground speed
  gubx->state.hmsl        = UBX_NAV_PVT_hMSL(gubx->msg_buf);
  SetBit(gubx->state.valid_fields, GPS_VALID_HMSL_BIT);
  gubx->state.gspeed      = UBX_NAV_PVT_gSpeed(gubx->msg_buf) / 10;

  // Copy NED velocities
  gubx->state.ned_vel.x   = UBX_NAV_PVT_velN(gubx->msg_buf) / 10;
  gubx->state.ned_vel.y   = UBX_NAV_PVT_velE(gubx->msg_buf) / 10;
  gubx->state.ned_vel.z   = UBX_NAV_PVT_velD(gubx->msg_buf) / 10;
  SetBit(gubx->state.valid_fields, GPS_VALID_VEL_NED_BIT);

  // Copy accuracy information
  gubx->state.pdop        = UBX_NAV_PVT_pDOP(gubx->msg_buf);
  gubx->state.hacc        = UBX_NAV_PVT_hAcc(gubx->msg_buf) / 10;
  gubx->state.vacc        = UBX_NAV_PVT_vAcc(gubx->msg_buf) / 10;
  gubx->state.sacc        = UBX_NAV_PVT_sAcc(gubx->msg_buf) / 10;

  if (!gubx->pacc_valid) {
    // workaround for PVT only
    gubx->state.pacc = gubx->state.hacc; // report horizontal accuracy
  }
}

static void gps_ubx_parse_nav_sol(struct GpsUbx *gubx)
{
  // Copy time and fix information
  uint8_t fix = UBX_NAV_SOL_gpsFix(gubx->msg_buf);
  if ((fix == GPS_FIX_3D && fix > gubx->state.fix) || fix < GPS_FIX_3D) {
    // update only if fix is better than current or fix not 3D
    // leaving fix if in GNSS or RTK mode
    gubx->state.fix = fix;
  }
  gubx->state.tow        = UBX_NAV_SOL_iTOW(gubx->msg_buf);
  gubx->state.week       = UBX_NAV_SOL_week(gubx->msg_buf);
  gubx->state.num_sv     = UBX_NAV_SOL_numSV(gubx->msg_buf);

  // Copy ecef position
  gubx->state.ecef_pos.x = UBX_NAV_SOL_ecefX(gubx->msg_buf);
  gubx->state.ecef_pos.y = UBX_NAV_SOL_ecefY(gubx->msg_buf);
  gubx->state.ecef_pos.z = UBX_NAV_SOL_ecefZ(gubx->msg_buf);
  SetBit(gubx->state.valid_fields, GPS_VALID_POS_ECEF_BIT);

  // Copy ecef velocity
  gubx->state.ecef_vel.x = UBX_NAV_SOL_ecefVX(gubx->msg_buf);
  gubx->state.ecef_vel.y = UBX_NAV_SOL_ecefVY(gubx->msg_buf);
  gubx->state.ecef_vel.z = UBX_NAV_SOL_ecefVZ(gubx->msg_buf);
  SetBit(gubx->state.valid_fields, GPS_VALID_VEL_ECEF_BIT);

  // Copy accuracy measurements
  gubx->state.pacc       = UBX_NAV_SOL_pAcc(gubx->msg_buf);
  gubx->state.sacc       = UBX_NAV_SOL_sAcc(gubx->msg_buf);
  gubx->state.pdop       = UBX_NAV_SOL_pDOP(gubx->msg_buf);
  gubx->pacc_valid = true;
}

static void gps_ubx_parse_nav_posecef(struct GpsUbx *gubx)
{
  gubx->state.tow        = UBX_NAV_POSECEF_iTOW(gubx->msg_buf);

  // Copy ecef position
  gubx->state.ecef_pos.x = UBX_NAV_POSECEF_ecefX(gubx->msg_buf);
  gubx->state.ecef_pos.y = UBX_NAV_POSECEF_ecefY(gubx->msg_buf);
  gubx->state.ecef_pos.z = UBX_NAV_POSECEF_ecefZ(gubx->msg_buf);
  SetBit(gubx->state.valid_fields, GPS_VALID_POS_ECEF_BIT);

  // Copy accuracy information
  gubx->state.pacc        = UBX_NAV_POSECEF_pAcc(gubx->msg_buf);
  gubx->pacc_valid = true;
}

static void gps_ubx_parse_nav_posllh(struct GpsUbx *gubx)
{
  // Copy LLA position
  gubx->state.lla_pos.lat = UBX_NAV_POSLLH_lat(gubx->msg_buf);
  gubx->state.lla_pos.lon = UBX_NAV_POSLLH_lon(gubx->msg_buf);
  gubx->state.lla_pos.alt = UBX_NAV_POSLLH_height(gubx->msg_buf);
  SetBit(gubx->state.valid_fields, GPS_VALID_POS_LLA_BIT);

  // Copy HMSL
  gubx->state.hmsl        = UBX_NAV_POSLLH_hMSL(gubx->msg_buf);
  SetBit(gubx->state.valid_fields, GPS_VALID_HMSL_BIT);

  // Copy accuracy information
  gubx->state.hacc        = UBX_NAV_POSLLH_hAcc(gubx->msg_buf) / 10;
  gubx->state.vacc        = UBX_NAV_POSLLH_vAcc(gubx->msg_buf) / 10;
}

static void gps_ubx_parse_nav_posutm(struct GpsUbx *gubx)
{
  uint8_t hem = UBX_NAV_POSUTM_hem(gubx->msg_buf);

  // Copy UTM state
  gubx->state.utm_pos.east = UBX_NAV_POSUTM_east(gubx->msg_buf);
  gubx->state.utm_pos.north = UBX_NAV_POSUTM_north(gubx->msg_buf);
  if (hem == UTM_HEM_SOUTH) {
    gubx->state.utm_pos.north -= 1000000000;  /* Subtract false northing: -10000km */
  }
  gubx->state.utm_pos.alt = UBX_NAV_POSUTM_alt(gubx->msg_buf) * 10;
  gubx->state.utm_pos.zone = UBX_NAV_POSUTM_zone(gubx->msg_buf);
  SetBit(gubx->state.valid_fields, GPS_VALID_POS_UTM_BIT);

  // Copy HMSL
  gubx->state.hmsl = gubx->state.utm_pos.alt;
  SetBit(gubx->state.valid_fields, GPS_VALID_HMSL_BIT);
}

static void gps_ubx_parse_velecef(struct GpsUbx *gubx)
{
  gubx->state.tow        = UBX_NAV_VELECEF_iTOW(gubx->msg_buf);

  // Copy ecef velocity
  gubx->state.ecef_vel.x = UBX_NAV_VELECEF_ecefVX(gubx->msg_buf);
  gubx->state.ecef_vel.y = UBX_NAV_VELECEF_ecefVY(gubx->msg_buf);
  gubx->state.ecef_vel.z = UBX_NAV_VELECEF_ecefVZ(gubx->msg_buf);
  SetBit(gubx->state.valid_fields, GPS_VALID_VEL_ECEF_BIT);

  // Copy accuracy measurements
  gubx->state.sacc       = UBX_NAV_VELECEF_sAcc(gubx->msg_buf);
}

static void gps_ubx_parse_nav_velned(struct GpsUbx *gubx)
{
  // Copy groundspeed and total 3d speed
  gubx->state.speed_3d = UBX_NAV_VELNED_speed(gubx->msg_buf);
  gubx->state.gspeed = UBX_NAV_VELNED_gSpeed(gubx->msg_buf);

  // Copy NED velocities
  gubx->state.ned_vel.x = UBX_NAV_VELNED_velN(gubx->msg_buf);
  gubx->state.ned_vel.y = UBX_NAV_VELNED_velE(gubx->msg_buf);
  gubx->state.ned_vel.z = UBX_NAV_VELNED_velD(gubx->msg_buf);
  SetBit(gubx->state.valid_fields, GPS_VALID_VEL_NED_BIT);

  // Copy course
  gubx->state.course = (RadOfDeg(UBX_NAV_VELNED_heading(gubx->msg_buf) * 10)) * 10;
  gubx->state.cacc = (RadOfDeg(UBX_NAV_VELNED_cAcc(gubx->msg_buf) * 10)) * 10;
  SetBit(gubx->state.valid_fields, GPS_VALID_COURSE_BIT);

  // Copy time of week
  gubx->state.tow = UBX_NAV_VELNED_iTOW(gubx->msg_buf);
}

static void gps_ubx_parse_nav_svinfo(struct GpsUbx *gubx)
{
  // Get the number of channels
  gubx->state.nb_channels = Min(UBX_NAV_SVINFO_numCh(gubx->msg_buf), GPS_NB_CHANNELS);

  // Go through all the different channels
  for (uint8_t i = 0; i < gubx->state.nb_channels; i++) {
    gubx->state.svinfos[i].svid = UBX_NAV_SVINFO_svid(gubx->msg_buf, i);
    gubx->state.svinfos[i].flags = UBX_NAV_SVINFO_flags(gubx->msg_buf, i);
    gubx->state.svinfos[i].qi = UBX_NAV_SVINFO_quality(gubx->msg_buf, i);
    gubx->state.svinfos[i].cno = UBX_NAV_SVINFO_cno(gubx->msg_buf, i);
    gubx->state.svinfos[i].elev = UBX_NAV_SVINFO_elev(gubx->msg_buf, i);
    gubx->state.svinfos[i].azim = UBX_NAV_SVINFO_azim(gubx->msg_buf, i);
  }
}

static void gps_ubx_parse_nav_sat(struct GpsUbx *gubx)
{
  // Get the number of channels(sattelites) and time of week
  gubx->state.tow         = UBX_NAV_SAT_iTOW(gubx->msg_buf);
  gubx->state.nb_channels = Min(UBX_NAV_SAT_numSvs(gubx->msg_buf), GPS_NB_CHANNELS);

  // Check the version
  uint8_t version = UBX_NAV_SAT_version(gubx->msg_buf);
  if (version != 1) {
    return;
  }

  // Go through all the different channels
  for (uint8_t i = 0; i < gubx->state.nb_channels; i++) {
    uint32_t flags = UBX_NAV_SVINFO_flags(gubx->msg_buf, i);
    gubx->state.svinfos[i].svid = UBX_NAV_SAT_svId(gubx->msg_buf, i);
    gubx->state.svinfos[i].cno = UBX_NAV_SAT_cno(gubx->msg_buf, i);
    gubx->state.svinfos[i].elev = UBX_NAV_SAT_elev(gubx->msg_buf, i);
    gubx->state.svinfos[i].azim = UBX_NAV_SAT_azim(gubx->msg_buf, i);
    gubx->state.svinfos[i].qi = flags & 0x7;
    gubx->state.svinfos[i].flags = (flags >> 3) & 0x1;
  }
}

static void gps_ubx_parse_nav_status(struct GpsUbx *gubx)
{
  uint8_t fix = UBX_NAV_STATUS_gpsFix(gubx->msg_buf);
  if ((fix == GPS_FIX_3D && fix > gubx->state.fix) || fix < GPS_FIX_3D) {
    // update only if fix is better than current or fix not 3D
    // leaving fix if in GNSS or RTK mode
    gubx->state.fix = fix;
  }
  gubx->state.tow     = UBX_NAV_STATUS_iTOW(gubx->msg_buf);
  gubx->status_flags  = UBX_NAV_STATUS_flags(gubx->msg_buf);
}

static void gps_ubx_parse_nav_relposned(struct GpsUbx *gubx)
{
#if USE_GPS_UBX_RTCM
  uint8_t version = UBX_NAV_RELPOSNED_version(gubx->msg_buf);
  if (version <= NAV_RELPOSNED_VERSION) {
    uint8_t flags       = UBX_NAV_RELPOSNED_flags(gubx->msg_buf);
    uint8_t carrSoln    = RTCMgetbitu(&flags, 3, 2);
    uint8_t relPosValid = RTCMgetbitu(&flags, 5, 1);
    uint8_t diffSoln    = RTCMgetbitu(&flags, 6, 1);
    uint8_t gnssFixOK   = RTCMgetbitu(&flags, 7, 1);

    /* Only save the latest valid relative position */
    if (relPosValid) {
      if (diffSoln && carrSoln == 2) {
        gubx->state.fix = 5; // rtk
      } else if(diffSoln && carrSoln == 1) {
        gubx->state.fix = 4; // dgnss
      } else if(gnssFixOK) {
        gubx->state.fix = 3; // 3D
      } else {
        gubx->state.fix = 0;
      }

      gps_relposned.iTOW          = UBX_NAV_RELPOSNED_iTOW(gubx->msg_buf);
      gps_relposned.refStationId  = UBX_NAV_RELPOSNED_refStationId(gubx->msg_buf);
      gps_relposned.relPosN     = UBX_NAV_RELPOSNED_relPosN(gubx->msg_buf);
      gps_relposned.relPosE     = UBX_NAV_RELPOSNED_relPosE(gubx->msg_buf);
      gps_relposned.relPosD     = UBX_NAV_RELPOSNED_relPosD(gubx->msg_buf) ;
      gps_relposned.relPosHPN   = UBX_NAV_RELPOSNED_relPosHPN(gubx->msg_buf);
      gps_relposned.relPosHPE   = UBX_NAV_RELPOSNED_relPosHPE(gubx->msg_buf);
      gps_relposned.relPosHPD   = UBX_NAV_RELPOSNED_relPosHPD(gubx->msg_buf);
      gps_relposned.relPosLength = UBX_NAV_RELPOSNED_relPosLength(gubx->msg_buf) / 100.f;
      gps_relposned.relPosHeading = UBX_NAV_RELPOSNED_relPosHeading(gubx->msg_buf) * 1e-5;
      gps_relposned.accN      = UBX_NAV_RELPOSNED_relPosHeading(gubx->msg_buf) * 1e-4;
      gps_relposned.accE      = UBX_NAV_RELPOSNED_relPosLength(gubx->msg_buf);
      gps_relposned.accD      = UBX_NAV_RELPOSNED_accD(gubx->msg_buf);
      gps_relposned.carrSoln    = carrSoln;
      gps_relposned.relPosValid = relPosValid;
      gps_relposned.diffSoln    = diffSoln;
      gps_relposned.gnssFixOK   = gnssFixOK;
    } else {
      gps_relposned.relPosHeading = NAN;
      gps_relposned.accN = 999;
    }
  }
#else
  (void)gubx;
#endif
}

void gps_ubx_read_message(struct GpsUbx *gubx);
void gps_ubx_read_message(struct GpsUbx *gubx)
{

  if (gubx->msg_class == UBX_NAV_ID) {
    switch (gubx->msg_id) {
      case UBX_NAV_POSECEF_ID:
        gps_ubx_parse_nav_posecef(gubx);
        break;
      case UBX_NAV_POSLLH_ID:
        gps_ubx_parse_nav_posllh(gubx);
        break;
      case UBX_NAV_STATUS_ID:
        gps_ubx_parse_nav_status(gubx);
        break;
      case UBX_NAV_SOL_ID:
        gps_ubx_parse_nav_sol(gubx);
        break;
      case UBX_NAV_PVT_ID:
        gps_ubx_parse_nav_pvt(gubx);
        break;
      case UBX_NAV_POSUTM_ID:
        gps_ubx_parse_nav_posutm(gubx);
        break;
      case UBX_NAV_VELECEF_ID:
        gps_ubx_parse_velecef(gubx);
        break;
      case UBX_NAV_VELNED_ID:
        gps_ubx_parse_nav_velned(gubx);
        break;
      case UBX_NAV_SVINFO_ID:
        gps_ubx_parse_nav_svinfo(gubx);
        break;
      case UBX_NAV_SAT_ID:
        gps_ubx_parse_nav_sat(gubx);
        break;
      case UBX_NAV_RELPOSNED_ID:
        gps_ubx_parse_nav_relposned(gubx);
        break;
      default:
        break;
    }
  }
  else if (gubx->msg_class == UBX_RXM_ID) {
    switch (gubx->msg_id) {
      // case UBX_RXM_RTCM_ID:
      //   gps_ubx_parse_rxm_rtcm(gubx);
      //   break;
      default:
        break;
    }
  }

}

#if LOG_RAW_GPS
#include "modules/loggers/sdlog_chibios.h"
#endif

/* UBX parsing */
void gps_ubx_parse(struct GpsUbx *gubx, uint8_t c)
{
#if LOG_RAW_GPS
  sdLogWriteByte(pprzLogFile, c);
#endif
  if (gubx->status < GOT_PAYLOAD) {
    gubx->ck_a += c;
    gubx->ck_b += gubx->ck_a;
  }
  switch (gubx->status) {
    case UNINIT:
      if (c == UBX_SYNC1) {
        gubx->status++;
      }
      break;
    case GOT_SYNC1:
      if (c != UBX_SYNC2) {
        gubx->error_last = GPS_UBX_ERR_OUT_OF_SYNC;
        goto error;
      }
      gubx->ck_a = 0;
      gubx->ck_b = 0;
      gubx->status++;
      break;
    case GOT_SYNC2:
      if (gubx->msg_available) {
        /* Previous message has not yet been parsed: discard this one */
        gubx->error_last = GPS_UBX_ERR_OVERRUN;
        goto error;
      }
      gubx->msg_class = c;
      gubx->status++;
      break;
    case GOT_CLASS:
      gubx->msg_id = c;
      gubx->status++;
      break;
    case GOT_ID:
      gubx->len = c;
      gubx->status++;
      break;
    case GOT_LEN1:
      gubx->len |= (c << 8);
      if (gubx->len > GPS_UBX_MAX_PAYLOAD) {
        gubx->error_last = GPS_UBX_ERR_MSG_TOO_LONG;
        goto error;
      }
      gubx->msg_idx = 0;
      gubx->status++;
      break;
    case GOT_LEN2:
      gubx->msg_buf[gubx->msg_idx] = c;
      gubx->msg_idx++;
      if (gubx->msg_idx >= gubx->len) {
        gubx->status++;
      }
      break;
    case GOT_PAYLOAD:
      if (c != gubx->ck_a) {
        gubx->error_last = GPS_UBX_ERR_CHECKSUM;
        goto error;
      }
      gubx->status++;
      break;
    case GOT_CHECKSUM1:
      if (c != gubx->ck_b) {
        gubx->error_last = GPS_UBX_ERR_CHECKSUM;
        goto error;
      }
      gubx->msg_available = true;
      goto restart;
      break;
    default:
      gubx->error_last = GPS_UBX_ERR_UNEXPECTED;
      goto error;
  }
  return;
error:
  gubx->error_cnt++;
restart:
  gubx->status = UNINIT;
  return;
}

static void ubx_send_1byte(struct link_device *dev, uint8_t byte)
{
  dev->put_byte(dev->periph, 0, byte);
  for(uint8_t i = 0; i < GPS_UBX_NB; i++) {
    if(gps_ubx[i].dev == dev) {
      gps_ubx[i].send_ck_a += byte;
      gps_ubx[i].send_ck_b += gps_ubx[i].send_ck_a;
      break;
    }
  }
}

void ubx_header(struct link_device *dev, uint8_t nav_id, uint8_t msg_id, uint16_t len)
{
  dev->put_byte(dev->periph, 0, UBX_SYNC1);
  dev->put_byte(dev->periph, 0, UBX_SYNC2);
  for(uint8_t i = 0; i < GPS_UBX_NB; i++) {
    if(gps_ubx[i].dev == dev) {
      gps_ubx[i].send_ck_a = 0;
      gps_ubx[i].send_ck_b = 0;
      break;
    }
  }
  ubx_send_1byte(dev, nav_id);
  ubx_send_1byte(dev, msg_id);
  ubx_send_1byte(dev, (uint8_t)(len & 0xFF));
  ubx_send_1byte(dev, (uint8_t)(len >> 8));
}

void ubx_trailer(struct link_device *dev)
{
  for(uint8_t i = 0; i < GPS_UBX_NB; i++) {
    if(gps_ubx[i].dev == dev) {
      dev->put_byte(dev->periph, 0, gps_ubx[i].send_ck_a);
      dev->put_byte(dev->periph, 0, gps_ubx[i].send_ck_b);
      break;
    }
  }
  dev->send_message(dev->periph, 0);
}

void ubx_send_bytes(struct link_device *dev, uint8_t len, uint8_t *bytes)
{
  int i;
  for (i = 0; i < len; i++) {
    ubx_send_1byte(dev, bytes[i]);
  }
}

void ubx_send_cfg_rst(struct link_device *dev, uint16_t bbr , UNUSED uint8_t reset_mode)
{
#ifdef UBX_GPS_PORT
  UbxSend_CFG_RST(dev, bbr, reset_mode, 0x00);
#endif /* else less harmful for HITL */
}

#ifndef GPS_UBX_UCENTER
#define gps_ubx_ucenter_event() {}
#else
#include "modules/gps/gps_ubx_ucenter.h"
#endif

void gps_ubx_msg(struct GpsUbx *gubx)
{
  // current timestamp
  uint32_t now_ts = get_sys_time_usec();

  gubx->state.last_msg_ticks = sys_time.nb_sec_rem;
  gubx->state.last_msg_time = sys_time.nb_sec;
  gps_ubx_read_message(gubx);
  gps_ubx_ucenter_event();
  if (gubx->msg_class == UBX_NAV_ID &&
      (gubx->msg_id == UBX_NAV_VELNED_ID ||
       gubx->msg_id == UBX_NAV_PVT_ID ||
       (gubx->msg_id == UBX_NAV_SOL_ID &&
        !bit_is_set(gubx->state.valid_fields, GPS_VALID_VEL_NED_BIT)))) {
    if (gubx->state.fix >= GPS_FIX_3D) {
      gubx->state.last_3dfix_ticks = sys_time.nb_sec_rem;
      gubx->state.last_3dfix_time = sys_time.nb_sec;
#ifdef GPS_LED
      LED_ON(GPS_LED);
    } else {
      LED_TOGGLE(GPS_LED);
    }
#else
    }
#endif
    AbiSendMsgGPS(gubx->state.comp_id, now_ts, &gubx->state);
  }
  gubx->msg_available = false;
}

void gps_ubx_periodic_check(void)
{
  for(uint8_t i = 0; i < GPS_UBX_NB; i++) {
    gps_periodic_check(&gps_ubx[i].state);
  }
}

/*
 * Write bytes to the ublox UART connection
 * This is a wrapper functions used in the librtcm library
 */
void gps_ublox_write(struct link_device *dev, uint8_t *buff, uint32_t n);
void gps_ublox_write(struct link_device *dev, uint8_t *buff, uint32_t n)
{
  uint32_t i = 0;
  for (i = 0; i < n; i++) {
    dev->put_byte(dev->periph, 0, buff[i]);
  }
  dev->send_message(dev->periph, 0);
  return;
}

/**
 * Override the default GPS packet injector to check the data before injection
 */
#if USE_GPS_UBX_RTCM
void gps_inject_data(uint8_t packet_id, uint8_t length, uint8_t *data)
{
  uint8_t i;

  // nothing to do
  if (length == 0) {
    return;
  }

#ifdef GPS_UBX_UCENTER
  // not ready
  if (gps_ubx_ucenter_get_status() != 0) {
    return;
  }
#endif

  // go through buffer
  for (i = 0; i < length; i++) {
    if (rtcm.nbyte == 0) {
      // wait for frame start byte
      if (data[i] == RTCM3_PREAMBLE) {
        rtcm.buff[rtcm.nbyte++] = data[i];
      }
    } else {
      // fill buffer
      if (rtcm.nbyte < INJECT_BUFF_SIZE) {
        rtcm.buff[rtcm.nbyte++] = data[i];
        if (rtcm.nbyte == 3) {
          // extract length
          rtcm.len = RTCMgetbitu(rtcm.buff, 14, 10) + 3;
        } else {
          // wait complete frame
          if (rtcm.nbyte == rtcm.len + 3) {
            // check CRC
            unsigned int crc1 = crc24q(rtcm.buff, rtcm.len);
            unsigned int crc2 = RTCMgetbitu(rtcm.buff, rtcm.len * 8, 24);

            if (crc1 == crc2)  {
              // write to GPS
#ifdef UBX_GPS_PORT
              gps_ublox_write(&(UBX_GPS_PORT).device, rtcm.buff, rtcm.len + 3);
#endif
              switch (packet_id) {
                case RTCM3_MSG_1005 : break;
                case RTCM3_MSG_1077 : break;
                case RTCM3_MSG_1087 : break;
                case RTCM3_MSG_4072 : break;
                case RTCM3_MSG_1230 : break;
                default: DEBUG_PRINT("Unknown type: %i", packet_id); break;
              }
            } else {
              DEBUG_PRINT("Skipping message %i (CRC failed) - %d", packet_id, rtcm.buff[0]);
              unsigned int j;
              for (j = 1; j < rtcm.len; j++) {
                DEBUG_PRINT(",%d", rtcm.buff[j]);
              }
              DEBUG_PRINT("\n");
            }

            // reset index
            rtcm.nbyte = 0;
          }
        }
      } else {
        // reset index
        rtcm.nbyte = 0;
      }
    }
  }
}

#endif /* USE_GPS_UBX_RTCM */
