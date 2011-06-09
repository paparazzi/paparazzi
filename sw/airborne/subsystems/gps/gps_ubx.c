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

#ifdef GPS_USE_LATLONG
#include "subsystems/nav.h"
#include "math/pprz_geodetic_float.h"
#endif

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


#define GpsUartSend1(c) GpsLink(Transmit(c))
#define GpsUartInitParam(_a,_b,_c) GpsLink(InitParam(_a,_b,_c))
#define GpsUartRunning GpsLink(TxRunning)
#define GpsUartSendMessage GpsLink(SendMessage)

#define UbxInitCheksum() { gps_ubx.send_ck_a = gps_ubx.send_ck_b = 0; }
#define UpdateChecksum(c) { gps_ubx.send_ck_a += c; gps_ubx.send_ck_b += gps_ubx.send_ck_a; }
#define UbxTrailer() { GpsUartSend1(gps_ubx.send_ck_a);  GpsUartSend1(gps_ubx.send_ck_b); GpsUartSendMessage(); }

#define UbxSend1(c) { uint8_t i8=c; GpsUartSend1(i8); UpdateChecksum(i8); }
#define UbxSend2(c) { uint16_t i16=c; UbxSend1(i16&0xff); UbxSend1(i16 >> 8); }
#define UbxSend1ByAddr(x) { UbxSend1(*x); }
#define UbxSend2ByAddr(x) { UbxSend1(*x); UbxSend1(*(x+1)); }
#define UbxSend4ByAddr(x) { UbxSend1(*x); UbxSend1(*(x+1)); UbxSend1(*(x+2)); UbxSend1(*(x+3)); }

#define UbxHeader(nav_id, msg_id, len) {        \
    GpsUartSend1(UBX_SYNC1);                    \
    GpsUartSend1(UBX_SYNC2);                    \
    UbxInitCheksum();                           \
    UbxSend1(nav_id);                           \
    UbxSend1(msg_id);                           \
    UbxSend2(len);                              \
  }


struct GpsUbx gps_ubx;

#ifdef GPS_CONFIGURE
bool_t gps_configuring;
static uint8_t gps_status_config;
#endif

void gps_impl_init(void) {
   gps_ubx.status = UNINIT;
   gps_ubx.msg_available = FALSE;
   gps_ubx.error_cnt = 0;
   gps_ubx.error_last = GPS_UBX_ERR_NONE;
#ifdef GPS_CONFIGURE
   gps_status_config = 0;
   gps_configuring = TRUE;
#endif
}


void gps_ubx_read_message(void) {

  if (gps_ubx.msg_class == UBX_NAV_ID) {
    if (gps_ubx.msg_id == UBX_NAV_SOL_ID) {
#ifdef GPS_TIMESTAMP
      /* get hardware clock ticks */
      SysTimeTimerStart(gps.t0);
      gps.t0_tow        = UBX_NAV_SOL_ITOW(gps_ubx.msg_buf);
      gps.t0_tow_frac   = UBX_NAV_SOL_Frac(gps_ubx.msg_buf);
#endif
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
      }
      else {
        LED_TOGGLE(GPS_LED);
      }
#endif
    } else if (gps_ubx.msg_id == UBX_NAV_POSLLH_ID) {
      gps.lla_pos.lat = RadOfDeg(UBX_NAV_POSLLH_LAT(gps_ubx.msg_buf));
      gps.lla_pos.lon = RadOfDeg(UBX_NAV_POSLLH_LON(gps_ubx.msg_buf));
      gps.lla_pos.alt = UBX_NAV_POSLLH_HEIGHT(gps_ubx.msg_buf);
      gps.hmsl        = UBX_NAV_POSLLH_HMSL(gps_ubx.msg_buf);
#ifdef GPS_USE_LATLONG
      /* Computes from (lat, long) in the referenced UTM zone */
      struct LlaCoor_f lla_f;
      lla_f.lat = ((float) gps.lla_pos.lat) / 1e7;
      lla_f.lon = ((float) gps.lla_pos.lon) / 1e7;
      struct UtmCoor_f utm_f;
      utm_f.zone = nav_utm_zone0;
      /* convert to utm */
      utm_of_lla_f(&utm_f, &lla_f);
      /* copy results of utm conversion */
      gps.utm_pos.east = utm_f.east*100;
      gps.utm_pos.north = utm_f.north*100;
      gps.utm_pos.alt = utm_f.alt*1000;
      gps.utm_pos.zone = nav_utm_zone0;
#else
    }
    else if (gps_ubx.msg_id == UBX_NAV_POSUTM_ID) {
      gps.utm_pos.east = UBX_NAV_POSUTM_EAST(gps_ubx.msg_buf);
      gps.utm_pos.north = UBX_NAV_POSUTM_NORTH(gps_ubx.msg_buf);
      uint8_t hem = UBX_NAV_POSUTM_HEM(gps_ubx.msg_buf);
      if (hem == UTM_HEM_SOUTH)
        gps.utm_pos.north -= 1000000000; /* Subtract false northing: -10000km */
      gps.utm_pos.alt = UBX_NAV_POSUTM_ALT(gps_ubx.msg_buf)*10;
      gps.hmsl = gps.utm_pos.alt;
      gps.lla_pos.alt = gps.utm_pos.alt; // FIXME: with UTM only you do not receive ellipsoid altitude
      gps.utm_pos.zone = UBX_NAV_POSUTM_ZONE(gps_ubx.msg_buf);
#endif
    }
    else if (gps_ubx.msg_id == UBX_NAV_VELNED_ID) {
      gps.speed_3d = UBX_NAV_VELNED_Speed(gps_ubx.msg_buf);
      gps.gspeed = UBX_NAV_VELNED_GSpeed(gps_ubx.msg_buf);
      gps.ned_vel.x = UBX_NAV_VELNED_VEL_N(gps_ubx.msg_buf);
      gps.ned_vel.y = UBX_NAV_VELNED_VEL_E(gps_ubx.msg_buf);
      gps.ned_vel.z = UBX_NAV_VELNED_VEL_D(gps_ubx.msg_buf);
      // Ublox gives I4 heading in 1e-5 degrees, apparenty from 0 to 360 degrees (not -180 to 180)
      // I4 max = 2^31 = 214 * 1e5 * 100 < 360 * 1e7: overflow on angles over 214 deg -> casted to -214 deg 
      // solution: First to radians, and then scale to 1e-7 radians
      // First x 10 for loosing less resolution, then to radians, then multiply x 10 again
      gps.course = (RadOfDeg(UBX_NAV_VELNED_Heading(gps_ubx.msg_buf)*10)) * 10; 
      gps.tow = UBX_NAV_VELNED_ITOW(gps_ubx.msg_buf);
    }
    else if (gps_ubx.msg_id == UBX_NAV_SVINFO_ID) {
      gps.nb_channels = Min(UBX_NAV_SVINFO_NCH(gps_ubx.msg_buf), GPS_NB_CHANNELS);
      uint8_t i;
      for(i = 0; i < gps.nb_channels; i++) {
        gps.svinfos[i].svid = UBX_NAV_SVINFO_SVID(gps_ubx.msg_buf, i);
        gps.svinfos[i].flags = UBX_NAV_SVINFO_Flags(gps_ubx.msg_buf, i);
        gps.svinfos[i].qi = UBX_NAV_SVINFO_QI(gps_ubx.msg_buf, i);
        gps.svinfos[i].cno = UBX_NAV_SVINFO_CNO(gps_ubx.msg_buf, i);
        gps.svinfos[i].elev = UBX_NAV_SVINFO_Elev(gps_ubx.msg_buf, i);
        gps.svinfos[i].azim = UBX_NAV_SVINFO_Azim(gps_ubx.msg_buf, i);
      }
    }
    else if (gps_ubx.msg_id == UBX_NAV_STATUS_ID) {
      gps.fix = UBX_NAV_STATUS_GPSfix(gps_ubx.msg_buf);
      gps_ubx.status_flags = UBX_NAV_STATUS_Flags(gps_ubx.msg_buf);
      gps_ubx.sol_flags = UBX_NAV_SOL_Flags(gps_ubx.msg_buf);
    }
  }
}


/* UBX parsing */
void gps_ubx_parse( uint8_t c ) {
  if (gps_ubx.status < GOT_PAYLOAD) {
    gps_ubx.ck_a += c;
    gps_ubx.ck_b += gps_ubx.ck_a;
  }
  switch (gps_ubx.status) {
  case UNINIT:
    if (c == UBX_SYNC1)
      gps_ubx.status++;
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
    gps_ubx.len |= (c<<8);
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



/*
 *
 *
 * GPS dynamic configuration
 *
 *
 */
#ifdef GPS_CONFIGURE

#define UBX_PROTO_MASK  0x0001
#define NMEA_PROTO_MASK 0x0002
#define RTCM_PROTO_MASK 0x0004

#define GPS_PORT_DDC   0x00
#define GPS_PORT_UART1 0x01
#define GPS_PORT_UART2 0x02
#define GPS_PORT_USB   0x03
#define GPS_PORT_SPI   0x04

#ifndef GPS_PORT_ID
#define GPS_PORT_ID GPS_PORT_UART1
#endif

#define __UBX_GPS_BAUD(_u) _u##_BAUD
#define _UBX_GPS_BAUD(_u) __UBX_GPS_BAUD(_u)
#define UBX_GPS_BAUD _UBX_GPS_BAUD(GPS_LINK)

/* Configure the GPS baud rate using the current uart baud rate. Busy
   wait for the end of the transmit. Then, BEFORE waiting for the ACK,
   change the uart rate. */
#if GPS_PORT_ID == GPS_PORT_UART1 || GPS_PORT_ID == GPS_PORT_UART2
void gps_configure_uart(void) {
#ifdef FMS_PERIODIC_FREQ
  UbxSend_CFG_PRT(GPS_PORT_ID, 0x0, 0x0, 0x000008D0, 38400, UBX_PROTO_MASK, UBX_PROTO_MASK, 0x0, 0x0);
  uint8_t loop=0;
  while (GpsUartRunning) {
    //doesn't work unless some printfs are used, so :
    if (loop<9) {
      printf("."); loop++;
    } else {
      printf("\b"); loop--;
    }
  }
#else
  UbxSend_CFG_PRT(GPS_PORT_ID, 0x0, 0x0, 0x000008D0, UBX_GPS_BAUD, UBX_PROTO_MASK, UBX_PROTO_MASK, 0x0, 0x0);
  while (GpsUartRunning); /* FIXME */
#endif

  GpsUartInitParam(UBX_GPS_BAUD,  UART_8N1, UART_FIFO_8);
}
#endif

#if GPS_PORT_ID == GPS_PORT_DDC
void gps_configure_uart(void) {
  UbxSend_CFG_PRT(GPS_PORT_ID, 0x0, 0x0, GPS_I2C_SLAVE_ADDR, 0x0, UBX_PROTO_MASK, UBX_PROTO_MASK, 0x0, 0x0);
}
#endif

#define IGNORED 0
#define RESERVED 0

#ifdef USER_GPS_CONFIGURE
#include USER_GPS_CONFIGURE
#else
static bool_t user_gps_configure(bool_t cpt) {
  switch (cpt) {
  case 0:
    //New ublox firmware v5 or higher uses CFG_NAV5 message, CFG_NAV is no longer available
    //UbxSend_CFG_NAV(NAV_DYN_AIRBORNE_2G, 3, 16, 24, 20, 5, 0, 0x3C, 0x3C, 0x14, 0x03E8 ,0x0000, 0x0, 0x17, 0x00FA, 0x00FA, 0x0064, 0x012C, 0x000F, 0x00, 0x00);
    UbxSend_CFG_NAV5(0x05, NAV5_DYN_AIRBORNE_2G, NAV5_3D_ONLY, IGNORED, IGNORED, IGNORED, IGNORED, IGNORED, IGNORED, IGNORED, IGNORED, IGNORED, RESERVED, RESERVED, RESERVED, RESERVED);
    break;
  case 1:
    UbxSend_CFG_MSG(UBX_NAV_ID, UBX_NAV_POSUTM_ID, 0, 1, 0, 0);
    break;
  case 2:
    UbxSend_CFG_MSG(UBX_NAV_ID, UBX_NAV_VELNED_ID, 0, 1, 0, 0);
    break;
  case 3:
    UbxSend_CFG_MSG(UBX_NAV_ID, UBX_NAV_STATUS_ID, 0, 1, 0, 0);
    break;
  case 4:
    UbxSend_CFG_MSG(UBX_NAV_ID, UBX_NAV_SVINFO_ID, 0, 4, 0, 0);
    break;
  case 5:
    UbxSend_CFG_MSG(UBX_NAV_ID, UBX_NAV_SOL_ID, 0, 8, 0, 0);
    break;
  case 6:
    UbxSend_CFG_SBAS(0x00, 0x00, 0x00, 0x00, 0x00);
    break;
  case 7:
    UbxSend_CFG_RATE(0x00FA, 0x0001, 0x0000);
    return FALSE;
  }
  return TRUE; /* Continue, except for the last case */
}
#endif // ! USER_GPS_CONFIGURE

/* GPS configuration. Must be called on ack message reception while
   gps_status_config < GPS_CONFIG_DONE */
void gps_configure( void ) {
  if (gps_ubx.msg_class == UBX_ACK_ID) {
    if (gps_ubx.msg_id == UBX_ACK_ACK_ID) {
      gps_status_config++;
    }
  }
  gps_configuring = user_gps_configure(gps_status_config);
}
#endif /* GPS_CONFIGURE */

void ubxsend_cfg_rst(uint16_t bbr , uint8_t reset_mode) {
#ifdef GPS_LINK
  UbxSend_CFG_RST(bbr, reset_mode, 0x00);
#endif /* else less harmful for HITL */
}


