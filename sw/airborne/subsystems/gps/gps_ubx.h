/*
 * Copyright (C) 2008-2011 The Paparazzi Team
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

/** @file gps_ubx.h
 * @brief UBX protocol specific code
 *
 */

#ifndef GPS_UBX_H
#define GPS_UBX_H

#ifdef GPS_CONFIGURE
#warning "Please use gps_ubx_ucenter.xml module instead of GPS_CONFIGURE"
#endif

#include "mcu_periph/uart.h"

/** Includes macros generated from ubx.xml */
#include "ubx_protocol.h"

#define GPS_NB_CHANNELS 16

#define GPS_UBX_MAX_PAYLOAD 255
struct GpsUbx {
  bool_t msg_available;
  uint8_t msg_buf[GPS_UBX_MAX_PAYLOAD] __attribute__ ((aligned));
  uint8_t msg_id;
  uint8_t msg_class;

  uint8_t status;
  uint16_t len;
  uint8_t msg_idx;
  uint8_t ck_a, ck_b;
  uint8_t send_ck_a, send_ck_b;
  uint8_t error_cnt;
  uint8_t error_last;

  uint8_t status_flags;
  uint8_t sol_flags;
  uint8_t have_velned;

};

extern struct GpsUbx gps_ubx;

#if USE_GPS_UBX_RXM_RAW
struct GpsUbxRawMes {
  double cpMes;
  double prMes;
  float doMes;
  uint8_t sv;
  int8_t mesQI;
  int8_t cno;
  uint8_t lli;
};

struct GpsUbxRaw {
  int32_t iTOW;
  int16_t week;
  uint8_t numSV;
  struct GpsUbxRawMes measures[GPS_NB_CHANNELS];
};

extern struct GpsUbxRaw gps_ubx_raw;
#endif

/*
 * This part is used by the autopilot to read data from a uart
 */
#define __GpsLink(dev, _x) dev##_x
#define _GpsLink(dev, _x)  __GpsLink(dev, _x)
#define GpsLink(_x) _GpsLink(GPS_LINK, _x)

#define GpsBuffer() GpsLink(ChAvailable())

#ifndef GPS_UBX_UCENTER
#define gps_ubx_ucenter_event() {}
#endif


/* Gps callback is called when receiving a VELNED or a SOL message
 * All position/speed messages are sent in one shot and VELNED is the last one on fixedwing
 * For rotorcraft, only SOL message is needed for pos/speed data
 */
#define GpsEvent(_sol_available_callback) {             \
    if (GpsBuffer()) {                                  \
      ReadGpsBuffer();                                  \
    }                                                   \
    if (gps_ubx.msg_available) {                        \
      gps.last_msg_ticks = sys_time.nb_sec_rem;         \
      gps.last_msg_time = sys_time.nb_sec;              \
      gps_ubx_read_message();                           \
      gps_ubx_ucenter_event();                          \
      if (gps_ubx.msg_class == UBX_NAV_ID &&            \
          (gps_ubx.msg_id == UBX_NAV_VELNED_ID ||       \
           (gps_ubx.msg_id == UBX_NAV_SOL_ID &&         \
            gps_ubx.have_velned == 0))) {               \
        if (gps.fix == GPS_FIX_3D) {                    \
          gps.last_3dfix_ticks = sys_time.nb_sec_rem;   \
          gps.last_3dfix_time = sys_time.nb_sec;        \
        }                                               \
        _sol_available_callback();                      \
      }                                                 \
      gps_ubx.msg_available = FALSE;                    \
    }                                                   \
  }

#define ReadGpsBuffer() {					\
    while (GpsLink(ChAvailable())&&!gps_ubx.msg_available)	\
      gps_ubx_parse(GpsLink(Getch()));			\
  }


extern void gps_ubx_read_message(void);
extern void gps_ubx_parse(uint8_t c);



/*
 * GPS Reset
 */

#define CFG_RST_Reset_Hardware 0x00
#define CFG_RST_Reset_Controlled 0x01
#define CFG_RST_Reset_Controlled_GPS_only 0x02
#define CFG_RST_Reset_Controlled_GPS_stop 0x08
#define CFG_RST_Reset_Controlled_GPS_start 0x09

#define CFG_RST_BBR_Hotstart  0x0000
#define CFG_RST_BBR_Warmstart 0x0001
#define CFG_RST_BBR_Coldstart 0xffff

extern void ubxsend_cfg_rst(uint16_t, uint8_t);

#define gps_ubx_Reset(_val) {                               \
    gps_ubx.reset = _val;                                       \
    if (gps_ubx.reset > CFG_RST_BBR_Warmstart)                  \
      gps_ubx.reset = CFG_RST_BBR_Coldstart;                    \
    ubxsend_cfg_rst(gps_ubx.reset, CFG_RST_Reset_Controlled);   \
  }

#define GpsUartSendMessage GpsLink(SendMessage)

#define GpsUartSend1(c) GpsLink(Transmit(c))
#define UbxInitCheksum() { gps_ubx.send_ck_a = gps_ubx.send_ck_b = 0; }
#define UbxUpdateChecksum(c) { gps_ubx.send_ck_a += c; gps_ubx.send_ck_b += gps_ubx.send_ck_a; }
#define UbxSend1(c) { uint8_t i8=c; GpsUartSend1(i8); UbxUpdateChecksum(i8); }
#define UbxSend2(c) { uint16_t i16=c; UbxSend1(i16&0xff); UbxSend1(i16 >> 8); }
#define UbxSend1ByAddr(x) { UbxSend1(*x); }
#define UbxSend2ByAddr(x) { UbxSend1(*x); UbxSend1(*(x+1)); }
#define UbxSend4ByAddr(x) { UbxSend1(*x); UbxSend1(*(x+1)); UbxSend1(*(x+2)); UbxSend1(*(x+3)); }
#define GpsUartSetBaudrate(_a) GpsLink(SetBaudrate(_a))

#define UbxHeader(nav_id, msg_id, len) {        \
    GpsUartSend1(UBX_SYNC1);                    \
    GpsUartSend1(UBX_SYNC2);                    \
    UbxInitCheksum();                           \
    UbxSend1(nav_id);                           \
    UbxSend1(msg_id);                           \
    UbxSend2(len);                              \
  }

#define UbxTrailer() { GpsUartSend1(gps_ubx.send_ck_a);  GpsUartSend1(gps_ubx.send_ck_b); GpsUartSendMessage(); }

#endif /* GPS_UBX_H */
