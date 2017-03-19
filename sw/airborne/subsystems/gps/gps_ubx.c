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


#include "subsystems/gps/gps_ubx.h"
#include "subsystems/abi.h"
#include "led.h"

#ifndef USE_GPS_UBX_RTCM
#define USE_GPS_UBX_RTCM 0
#endif


#if USE_GPS_UBX_RTCM
#include "subsystems/gps/librtcm3/CRC24Q.h"
#define RTCM3_PREAMBLE 0xD3
#define RTCM3_MSG_1005 0x69
#define RTCM3_MSG_1077 0xB1
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
#define NAV_RELPOSNED_VERSION   0x00
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

#if USE_GPS_UBX_RTCM
extern struct GpsRelposNED gps_relposned;
extern struct RtcmMan rtcm_man;

#ifndef INJECT_BUFF_SIZE
#define INJECT_BUFF_SIZE 512
#endif
/* RTCM control struct type */
struct rtcm_t {
  uint32_t nbyte;                   ///< number of bytes in message buffer
  uint32_t len;                     ///< message length (bytes)
  uint8_t buff[INJECT_BUFF_SIZE+1]; ///< message buffer
};
struct rtcm_t rtcm = { 0 };

#endif

bool safeToInject = true;
struct GpsTimeSync gps_ubx_time_sync;

void gps_ubx_init(void)
{
  gps_ubx.status = UNINIT;
  gps_ubx.msg_available = false;
  gps_ubx.error_cnt = 0;
  gps_ubx.error_last = GPS_UBX_ERR_NONE;

  gps_ubx.state.comp_id = GPS_UBX_ID;
}

void gps_ubx_event(void)
{
  struct link_device *dev = &((UBX_GPS_LINK).device);

  while (dev->char_available(dev->periph)) {
    gps_ubx_parse(dev->get_byte(dev->periph));
    if (gps_ubx.msg_available) {
      gps_ubx_msg();
    }
  }
}

void gps_ubx_read_message(void)
{

  if (gps_ubx.msg_class == UBX_NAV_ID) {
    if (gps_ubx.msg_id == UBX_NAV_SOL_ID) {
      /* get hardware clock ticks */
      gps_ubx_time_sync.t0_ticks      = sys_time.nb_tick;
      gps_ubx_time_sync.t0_tow        = UBX_NAV_SOL_ITOW(gps_ubx.msg_buf);
      gps_ubx_time_sync.t0_tow_frac   = UBX_NAV_SOL_Frac(gps_ubx.msg_buf);
      gps_ubx.state.tow        = UBX_NAV_SOL_ITOW(gps_ubx.msg_buf);
      gps_ubx.state.week       = UBX_NAV_SOL_week(gps_ubx.msg_buf);
#if ! USE_GPS_UBX_RTCM
      gps_ubx.state.fix        = UBX_NAV_SOL_GPSfix(gps_ubx.msg_buf);
#endif
      gps_ubx.state.ecef_pos.x = UBX_NAV_SOL_ECEF_X(gps_ubx.msg_buf);
      gps_ubx.state.ecef_pos.y = UBX_NAV_SOL_ECEF_Y(gps_ubx.msg_buf);
      gps_ubx.state.ecef_pos.z = UBX_NAV_SOL_ECEF_Z(gps_ubx.msg_buf);
      SetBit(gps_ubx.state.valid_fields, GPS_VALID_POS_ECEF_BIT);
      gps_ubx.state.pacc       = UBX_NAV_SOL_Pacc(gps_ubx.msg_buf);
      gps_ubx.state.ecef_vel.x = UBX_NAV_SOL_ECEFVX(gps_ubx.msg_buf);
      gps_ubx.state.ecef_vel.y = UBX_NAV_SOL_ECEFVY(gps_ubx.msg_buf);
      gps_ubx.state.ecef_vel.z = UBX_NAV_SOL_ECEFVZ(gps_ubx.msg_buf);
      SetBit(gps_ubx.state.valid_fields, GPS_VALID_VEL_ECEF_BIT);
      gps_ubx.state.sacc       = UBX_NAV_SOL_Sacc(gps_ubx.msg_buf);
      gps_ubx.state.pdop       = UBX_NAV_SOL_PDOP(gps_ubx.msg_buf);
      gps_ubx.state.num_sv     = UBX_NAV_SOL_numSV(gps_ubx.msg_buf);
#ifdef GPS_LED
      if (gps_ubx.state.fix == GPS_FIX_3D) {
        LED_ON(GPS_LED);
      } else {
        LED_TOGGLE(GPS_LED);
      }
#endif
    } else if (gps_ubx.msg_id == UBX_NAV_POSLLH_ID) {
      gps_ubx.state.lla_pos.lat = UBX_NAV_POSLLH_LAT(gps_ubx.msg_buf);
      gps_ubx.state.lla_pos.lon = UBX_NAV_POSLLH_LON(gps_ubx.msg_buf);
      gps_ubx.state.lla_pos.alt = UBX_NAV_POSLLH_HEIGHT(gps_ubx.msg_buf);
      SetBit(gps_ubx.state.valid_fields, GPS_VALID_POS_LLA_BIT);
      gps_ubx.state.hmsl        = UBX_NAV_POSLLH_HMSL(gps_ubx.msg_buf);
      SetBit(gps_ubx.state.valid_fields, GPS_VALID_HMSL_BIT);
    } else if (gps_ubx.msg_id == UBX_NAV_POSUTM_ID) {
      gps_ubx.state.utm_pos.east = UBX_NAV_POSUTM_EAST(gps_ubx.msg_buf);
      gps_ubx.state.utm_pos.north = UBX_NAV_POSUTM_NORTH(gps_ubx.msg_buf);
      uint8_t hem = UBX_NAV_POSUTM_HEM(gps_ubx.msg_buf);
      if (hem == UTM_HEM_SOUTH) {
        gps_ubx.state.utm_pos.north -= 1000000000;  /* Subtract false northing: -10000km */
      }
      gps_ubx.state.utm_pos.alt = UBX_NAV_POSUTM_ALT(gps_ubx.msg_buf) * 10;
      gps_ubx.state.utm_pos.zone = UBX_NAV_POSUTM_ZONE(gps_ubx.msg_buf);
      SetBit(gps_ubx.state.valid_fields, GPS_VALID_POS_UTM_BIT);

      gps_ubx.state.hmsl = gps_ubx.state.utm_pos.alt;
      SetBit(gps_ubx.state.valid_fields, GPS_VALID_HMSL_BIT);
    } else if (gps_ubx.msg_id == UBX_NAV_VELNED_ID) {
      gps_ubx.state.speed_3d = UBX_NAV_VELNED_Speed(gps_ubx.msg_buf);
      gps_ubx.state.gspeed = UBX_NAV_VELNED_GSpeed(gps_ubx.msg_buf);
      gps_ubx.state.ned_vel.x = UBX_NAV_VELNED_VEL_N(gps_ubx.msg_buf);
      gps_ubx.state.ned_vel.y = UBX_NAV_VELNED_VEL_E(gps_ubx.msg_buf);
      gps_ubx.state.ned_vel.z = UBX_NAV_VELNED_VEL_D(gps_ubx.msg_buf);
      SetBit(gps_ubx.state.valid_fields, GPS_VALID_VEL_NED_BIT);
      // Ublox gives I4 heading in 1e-5 degrees, apparenty from 0 to 360 degrees (not -180 to 180)
      // I4 max = 2^31 = 214 * 1e5 * 100 < 360 * 1e7: overflow on angles over 214 deg -> casted to -214 deg
      // solution: First to radians, and then scale to 1e-7 radians
      // First x 10 for loosing less resolution, then to radians, then multiply x 10 again
      gps_ubx.state.course = (RadOfDeg(UBX_NAV_VELNED_Heading(gps_ubx.msg_buf) * 10)) * 10;
      SetBit(gps_ubx.state.valid_fields, GPS_VALID_COURSE_BIT);
      gps_ubx.state.cacc = (RadOfDeg(UBX_NAV_VELNED_CAcc(gps_ubx.msg_buf) * 10)) * 10;
      gps_ubx.state.tow = UBX_NAV_VELNED_ITOW(gps_ubx.msg_buf);
    } else if (gps_ubx.msg_id == UBX_NAV_SVINFO_ID) {
      gps_ubx.state.nb_channels = Min(UBX_NAV_SVINFO_NCH(gps_ubx.msg_buf), GPS_NB_CHANNELS);
      uint8_t i;
      for (i = 0; i < gps_ubx.state.nb_channels; i++) {
        gps_ubx.state.svinfos[i].svid = UBX_NAV_SVINFO_SVID(gps_ubx.msg_buf, i);
        gps_ubx.state.svinfos[i].flags = UBX_NAV_SVINFO_Flags(gps_ubx.msg_buf, i);
        gps_ubx.state.svinfos[i].qi = UBX_NAV_SVINFO_QI(gps_ubx.msg_buf, i);
        gps_ubx.state.svinfos[i].cno = UBX_NAV_SVINFO_CNO(gps_ubx.msg_buf, i);
        gps_ubx.state.svinfos[i].elev = UBX_NAV_SVINFO_Elev(gps_ubx.msg_buf, i);
        gps_ubx.state.svinfos[i].azim = UBX_NAV_SVINFO_Azim(gps_ubx.msg_buf, i);
      }
    } else if (gps_ubx.msg_id == UBX_NAV_STATUS_ID) {
#if !USE_GPS_UBX_RTCM
      gps_ubx.state.fix = UBX_NAV_STATUS_GPSfix(gps_ubx.msg_buf);
#endif
      gps_ubx.status_flags = UBX_NAV_STATUS_Flags(gps_ubx.msg_buf);
      gps_ubx.sol_flags = UBX_NAV_SOL_Flags(gps_ubx.msg_buf);
    } else if (gps_ubx.msg_id == UBX_NAV_RELPOSNED_ID) {
#if USE_GPS_UBX_RTCM
      uint8_t version = UBX_NAV_RELPOSNED_VERSION(gps_ubx.msg_buf);
      if (version == NAV_RELPOSNED_VERSION) {
        gps_relposned.iTOW          = UBX_NAV_RELPOSNED_ITOW(gps_ubx.msg_buf);
        gps_relposned.refStationId  = UBX_NAV_RELPOSNED_refStationId(gps_ubx.msg_buf);
        gps_relposned.relPosN     = UBX_NAV_RELPOSNED_RELPOSN(gps_ubx.msg_buf);
        gps_relposned.relPosE     = UBX_NAV_RELPOSNED_RELPOSE(gps_ubx.msg_buf);
        gps_relposned.relPosD     = UBX_NAV_RELPOSNED_RELPOSD(gps_ubx.msg_buf) ;
        gps_relposned.relPosHPN   = UBX_NAV_RELPOSNED_RELPOSNHP(gps_ubx.msg_buf);
        gps_relposned.relPosHPE   = UBX_NAV_RELPOSNED_RELPOSEHP(gps_ubx.msg_buf);
        gps_relposned.relPosHPD   = UBX_NAV_RELPOSNED_RELPOSDHP(gps_ubx.msg_buf);
        gps_relposned.accN      = UBX_NAV_RELPOSNED_Nacc(gps_ubx.msg_buf);
        gps_relposned.accE      = UBX_NAV_RELPOSNED_Eacc(gps_ubx.msg_buf);
        gps_relposned.accD      = UBX_NAV_RELPOSNED_Dacc(gps_ubx.msg_buf);
        uint8_t flags           = UBX_NAV_RELPOSNED_Flags(gps_ubx.msg_buf);
        gps_relposned.carrSoln    = RTCMgetbitu(&flags, 3, 2);
        gps_relposned.relPosValid   = RTCMgetbitu(&flags, 5, 1);
        gps_relposned.diffSoln    = RTCMgetbitu(&flags, 6, 1);
        gps_relposned.gnssFixOK   = RTCMgetbitu(&flags, 7, 1);
        if (gps_relposned.gnssFixOK > 0) {
          if (gps_relposned.diffSoln > 0) {
            if (gps_relposned.carrSoln == 2) {
              gps_ubx.state.fix = 5; // rtk
            } else {
              gps_ubx.state.fix = 4; // dgnss
            }
          } else {
            gps_ubx.state.fix = 3; // 3D
          }
        } else {
          gps_ubx.state.fix = 0;
        }
        DEBUG_PRINT("GNSS Fix OK: %i\tDGNSS: %i\tRTK: %i\trelPosValid: %i\trefStationId: %i\n", gps_relposned.gnssFixOK,
                    gps_relposned.diffSoln, gps_relposned.carrSoln, gps_relposned.relPosValid, gps_relposned.refStationId);
      }
#endif // USE_GPS_UBX_RTCM
    }
  } else if (gps_ubx.msg_class == UBX_RXM_ID) {
    if (gps_ubx.msg_id == UBX_RXM_RAW_ID) {
#if USE_GPS_UBX_RXM_RAW
      gps_ubx_raw.iTOW = UBX_RXM_RAW_iTOW(gps_ubx.msg_buf);
      gps_ubx_raw.week = UBX_RXM_RAW_week(gps_ubx.msg_buf);
      gps_ubx_raw.numSV = UBX_RXM_RAW_numSV(gps_ubx.msg_buf);
      uint8_t i;
      uint8_t max_SV = Min(gps_ubx_raw.numSV, GPS_UBX_NB_CHANNELS);
      for (i = 0; i < max_SV; i++) {
        gps_ubx_raw.measures[i].cpMes = UBX_RXM_RAW_cpMes(gps_ubx.msg_buf, i);
        gps_ubx_raw.measures[i].prMes = UBX_RXM_RAW_prMes(gps_ubx.msg_buf, i);
        gps_ubx_raw.measures[i].doMes = UBX_RXM_RAW_doMes(gps_ubx.msg_buf, i);
        gps_ubx_raw.measures[i].sv = UBX_RXM_RAW_sv(gps_ubx.msg_buf, i);
        gps_ubx_raw.measures[i].mesQI = UBX_RXM_RAW_mesQI(gps_ubx.msg_buf, i);
        gps_ubx_raw.measures[i].cno = UBX_RXM_RAW_cno(gps_ubx.msg_buf, i);
        gps_ubx_raw.measures[i].lli = UBX_RXM_RAW_lli(gps_ubx.msg_buf, i);
      }
#endif // USE_GPS_UBX_RXM_RAW
    } else if (gps_ubx.msg_id == UBX_RXM_RTCM_ID) {
#if USE_GPS_UBX_RTCM
      uint8_t version   = UBX_RXM_RTCM_version(gps_ubx.msg_buf);
      if (version == RXM_RTCM_VERSION) {
        //      uint8_t flags     = UBX_RXM_RTCM_flags(gps_ubx.msg_buf);
        //      bool crcFailed    = RTCMgetbitu(&flags, 7, 1);
        //      uint16_t refStation = UBX_RXM_RTCM_refStation(gps_ubx.msg_buf);
        //      uint16_t msgType  = UBX_RXM_RTCM_msgType(gps_ubx.msg_buf);
        //      DEBUG_PRINT("Message %i from refStation %i processed (CRCfailed: %i)\n", msgType, refStation, crcFailed);

        rtcm_man.RefStation  = UBX_RXM_RTCM_refStation(gps_ubx.msg_buf);
        rtcm_man.MsgType     = UBX_RXM_RTCM_msgType(gps_ubx.msg_buf);
        uint8_t flags     = UBX_RXM_RTCM_flags(gps_ubx.msg_buf);
        bool crcFailed    = RTCMgetbitu(&flags, 7, 1);
        switch (rtcm_man.MsgType) {
          case 1005:
            rtcm_man.Cnt105 += 1;
            rtcm_man.Crc105 += crcFailed;
            break;
          case 1077:
            rtcm_man.Cnt177 += 1;
            rtcm_man.Crc177 += crcFailed;;
            break;
          case 1087:
            rtcm_man.Cnt187 += 1;
            rtcm_man.Crc187 += crcFailed;;
            break;
          default:
            break;
        }
      } else {
        DEBUG_PRINT("Unknown RXM_RTCM version: %i\n", version);
      }
#endif // USE_GPS_UBX_RTCM
    }
  }
}

#if LOG_RAW_GPS
#include "modules/loggers/sdlog_chibios.h"
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
      gps_ubx.msg_available = true;
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
  dev->put_byte(dev->periph, 0, byte);
  gps_ubx.send_ck_a += byte;
  gps_ubx.send_ck_b += gps_ubx.send_ck_a;
}

void ubx_header(struct link_device *dev, uint8_t nav_id, uint8_t msg_id, uint16_t len)
{
  dev->put_byte(dev->periph, 0, UBX_SYNC1);
  dev->put_byte(dev->periph, 0, UBX_SYNC2);
  gps_ubx.send_ck_a = 0;
  gps_ubx.send_ck_b = 0;
  ubx_send_1byte(dev, nav_id);
  ubx_send_1byte(dev, msg_id);
  ubx_send_1byte(dev, (uint8_t)(len&0xFF));
  ubx_send_1byte(dev, (uint8_t)(len>>8));
}

void ubx_trailer(struct link_device *dev)
{
  dev->put_byte(dev->periph, 0, gps_ubx.send_ck_a);
  dev->put_byte(dev->periph, 0, gps_ubx.send_ck_b);
  dev->send_message(dev->periph, 0);
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
#ifdef UBX_GPS_LINK
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

  gps_ubx.state.last_msg_ticks = sys_time.nb_sec_rem;
  gps_ubx.state.last_msg_time = sys_time.nb_sec;
  gps_ubx_read_message();
  gps_ubx_ucenter_event();
  if (gps_ubx.msg_class == UBX_NAV_ID &&
      (gps_ubx.msg_id == UBX_NAV_VELNED_ID ||
       (gps_ubx.msg_id == UBX_NAV_SOL_ID &&
        !bit_is_set(gps_ubx.state.valid_fields, GPS_VALID_VEL_NED_BIT)))) {
    if (gps_ubx.state.fix >= GPS_FIX_3D) {
      gps_ubx.state.last_3dfix_ticks = sys_time.nb_sec_rem;
      gps_ubx.state.last_3dfix_time = sys_time.nb_sec;
    }
    AbiSendMsgGPS(GPS_UBX_ID, now_ts, &gps_ubx.state);
  }
  gps_ubx.msg_available = false;
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
 * Override the default GPS packet injector to inject the data
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
              gps_ublox_write(&(UBX_GPS_LINK).device, rtcm.buff, rtcm.len + 3);
              switch (packet_id) {
                case RTCM3_MSG_1005 : break;
                case RTCM3_MSG_1077 : break;
                case RTCM3_MSG_1087 : break;
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

#endif
