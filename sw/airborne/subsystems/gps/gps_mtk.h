/*
 * Copyright (C) 2011 The Paparazzi Team
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


#ifndef MTK_H
#define MTK_H

#include "mcu_periph/uart.h"

/** Includes macros generated from mtk.xml */
#include "mtk_protocol.h"

#define GPS_MTK_MAX_PAYLOAD 255

struct GpsMtk {
  bool_t msg_available;
  uint8_t msg_buf[GPS_MTK_MAX_PAYLOAD] __attribute__ ((aligned));
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
};

extern struct GpsMtk gps_mtk;


/*
 * This part is used by the autopilot to read data from a uart
 */
#define __GpsLink(dev, _x) dev##_x
#define _GpsLink(dev, _x)  __GpsLink(dev, _x)
#define GpsLink(_x) _GpsLink(GPS_LINK, _x)

#define GpsBuffer() GpsLink(ChAvailable())

#ifdef GPS_CONFIGURE
extern bool_t gps_configuring;
#define GpsConfigure() {            \
    if (gps_configuring)            \
      gps_configure();              \
  }
#else
#define GpsConfigure() {}
#endif

#define GpsEvent(_sol_available_callback) {         \
    if (GpsBuffer()) {                              \
      ReadGpsBuffer();                              \
      GpsConfigure();                               \
    }                                               \
    if (gps_mtk.msg_available) {                    \
      gps_mtk_read_message();                       \
      if (gps_mtk.msg_class == MTK_DIY14_ID &&      \
          gps_mtk.msg_id == MTK_DIY14_NAV_ID) {     \
        if (gps.fix == GPS_FIX_3D) {                \
          gps.last_fix_time = cpu_time_sec;         \
        }                                           \
        _sol_available_callback();                  \
      }                                             \
      if (gps_mtk.msg_class == MTK_DIY16_ID &&      \
          gps_mtk.msg_id == MTK_DIY16_NAV_ID) {     \
        if (gps.fix == GPS_FIX_3D) {                \
          gps.last_fix_ticks = cpu_time_ticks;      \
          gps.last_fix_time = cpu_time_sec;         \
        }                                           \
        _sol_available_callback();                  \
      }                                             \
      gps_mtk.msg_available = FALSE;                \
    }                                               \
  }

#define ReadGpsBuffer() {					\
    while (GpsLink(ChAvailable())&&!gps_mtk.msg_available)	\
      gps_mtk_parse(GpsLink(Getch()));			\
  }


extern void gps_mtk_read_message(void);
extern void gps_mtk_parse(uint8_t c);

#define MTK_DIY_FIX_3D      3
#define MTK_DIY_FIX_2D      2
#define MTK_DIY_FIX_NONE    1

/*
 * dynamic GPS configuration
 */
#ifdef GPS_CONFIGURE
#define MTK_DIY_SET_BINARY  "$PGCMD,16,0,0,0,0,0*6A\r\n"
#define MTK_DIY_SET_NMEA    "$PGCMD,16,1,1,1,1,1*6B\r\n"

#define MTK_DIY_OUTPUT_1HZ  "$PMTK220,1000*1F\r\n"
#define MTK_DIY_OUTPUT_2HZ  "$PMTK220,500*2B\r\n"
#define MTK_DIY_OUTPUT_4HZ  "$PMTK220,250*29\r\n"
#define MTK_DIY_OTUPUT_5HZ  "$PMTK220,200*2C\r\n"
#define MTK_DIY_OUTPUT_10HZ "$PMTK220,100*2F\r\n"

#define MTK_BAUD_RATE_38400 "$PMTK251,38400*27\r\n"

#define MTK_DIY_SBAS_ON     "$PMTK313,1*2E\r\n"
#define MTK_DIY_SBAS_OFF    "$PMTK313,0*2F\r\n"

#define MTK_DIY_WAAS_ON     "$PSRF151,1*3F\r\n"
#define MTK_DIY_WAAS_OFF    "$PSRF151,0*3E\r\n"

extern void gps_configure(void);
extern void gps_configure_uart(void);
#endif

#endif /* MTK_H */
