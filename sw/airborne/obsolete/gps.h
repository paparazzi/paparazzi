/*
 * $Id$
 *
 * Copyright (C) 2003  Pascal Brisset, Antoine Drouin
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
 *
 */

/** \file gps.h
 *  \brief Device independent GPS code
 *
*/

#ifndef GPS_H
#define GPS_H

#include "std.h"
#include "gps_hw.h"

#ifdef GPS
#error "The flag GPS has been deprecated. Please replace it with USE_GPS."
#endif

#ifdef UBX
#include "gps_ubx.h"
#elif defined USE_GPS_XSENS
#include "gps_xsens.h"
#elif defined SITL
#define GPS_NB_CHANNELS 16
#define GpsFixValid() (gps_mode == 3)
#elif defined UGEAR
#define GPS_NB_CHANNELS 16
#else
#define GPS_NB_CHANNELS 1
#endif


extern uint8_t gps_mode; /* Receiver status */
extern uint8_t gps_status_flags;
extern uint8_t gps_sol_flags;
extern uint16_t gps_week;    /* weeks */
extern uint32_t gps_itow;    /* ms */
extern int32_t  gps_alt;    /* above MSL in cm       */
extern uint16_t gps_speed_3d;  /* cm/s     */
extern uint16_t gps_gspeed;  /* cm/s     */
extern int16_t  gps_climb;  /* m/s     */
extern int16_t  gps_course; /* decideg     */
extern int32_t gps_utm_east, gps_utm_north; /** cm */
extern uint8_t gps_utm_zone;
extern int32_t gps_lat, gps_lon; /* 1e7 deg */
extern int32_t gps_hmsl;
extern uint16_t gps_PDOP;
extern uint32_t gps_Pacc, gps_Sacc;
extern int32_t gps_ecefVZ;
extern uint8_t gps_numSV;
extern uint8_t gps_configuring;

extern uint16_t last_gps_msg_t; /** cputime of the last gps message */
extern bool_t gps_verbose_downlink;

void gps_init( void );
void gps_configure( void );
void parse_gps_msg( void );
void gps_downlink( void );
void gps_send( void );
void gps_configure_uart( void );


extern volatile uint8_t gps_msg_received;
extern bool_t gps_pos_available;
extern uint8_t gps_nb_ovrn;

#ifdef UGEAR
extern volatile uint16_t gps_msg_received_counter;
#endif

/** Number of scanned satellites */
extern uint8_t gps_nb_channels;

/** Space Vehicle Information */
struct svinfo {
  uint8_t svid;
  uint8_t flags;
  uint8_t qi;
  uint8_t cno;
  int8_t elev; /** deg */
  int16_t azim; /** deg */
};

extern struct svinfo gps_svinfos[GPS_NB_CHANNELS];

#ifndef SITL
#include "mcu_periph/uart.h"

#define __GpsLink(dev, _x) dev##_x
#define _GpsLink(dev, _x)  __GpsLink(dev, _x)
#define GpsLink(_x) _GpsLink(GPS_LINK, _x)

#define GpsBuffer() GpsLink(ChAvailable())
#define ReadGpsBuffer() { while (GpsLink(ChAvailable())&&!gps_msg_received) parse_ubx(GpsLink(Getch())); }
#define GpsUartSend1(c) GpsLink(Transmit(c))
#define GpsUartSetBaudrate(_b) GpsLink(SetBaudrate(_b))
#define GpsUartRunning GpsLink(TxRunning)
#define GpsUartSendMessage GpsLink(SendMessage)


#endif /** !SITL */

#ifdef GPS_LED
#define GpsToggleLed() LED_TOGGLE(GPS_LED)
#else
#define GpsToggleLed() {}
#endif

#if defined(USE_GPS) || defined(USE_GPS_XSENS) || defined(SITL)
#  define GpsTimeoutError (cpu_time_sec - last_gps_msg_t > FAILSAFE_DELAY_WITHOUT_GPS)
#else
#  define GpsTimeoutError 1
#endif

#define UseGpsPosNoSend(_callback) {			\
    if (GpsFixValid()) {			\
      last_gps_msg_t = cpu_time_sec;		\
      _callback();				\
      GpsToggleLed();				\
    }						\
  }

#ifdef GPS_CONFIGURE
#define GpsParseOrConfigure() {			\
    if (gps_configuring)			\
      gps_configure();				\
    else					\
      parse_gps_msg();				\
  }
#else
#define GpsParseOrConfigure() parse_gps_msg()
#endif


#define GpsEventCheckAndHandle(_callback, _verbose) {   \
    if (GpsBuffer()) {                                  \
      ReadGpsBuffer();                                  \
    }                                                   \
    if (gps_msg_received) {                             \
      GpsParseOrConfigure();                            \
      gps_msg_received = FALSE;                         \
      if (gps_pos_available) {                          \
        gps_verbose_downlink = _verbose;                \
        UseGpsPos(_callback);                           \
        gps_pos_available = FALSE;                      \
      }                                                 \
    }                                                   \
  }


#endif /* GPS_H */
