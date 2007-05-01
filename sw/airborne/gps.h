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

#ifdef UBX
#include "gps_ubx.h"
#elif defined SITL
#define GPS_NB_CHANNELS 16
#define GpsFixValid() (gps_mode == 3)
#else
#define GPS_NB_CHANNELS 1
#endif


extern uint8_t gps_mode; /* Receiver status */
extern uint32_t gps_itow;    /* ms */
extern int32_t  gps_alt;    /* cm       */
extern uint16_t gps_gspeed;  /* cm/s     */
extern int16_t  gps_climb;  /* m/s     */
extern int16_t  gps_course; /* decideg     */
extern int32_t gps_utm_east, gps_utm_north; /** cm */
extern uint8_t gps_utm_zone;

extern int32_t gps_lat, gps_lon; /* 1e-7 deg */
extern uint16_t last_gps_msg_t; /** cputime of the last gps message */

void gps_init( void );
void gps_configure( void );
void parse_gps_msg( void );
void estimator_update_state_gps( void );
void use_gps_pos( void );

extern volatile uint8_t gps_msg_received;
extern bool_t gps_pos_available;
extern uint8_t gps_nb_ovrn;

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
#include "uart.h"

#define __GpsLink(dev, _x) dev##_x
#define _GpsLink(dev, _x)  __GpsLink(dev, _x)
#define GpsLink(_x) _GpsLink(GPS_LINK, _x)

#define GpsBuffer() GpsLink(ChAvailable())
#define ReadGpsBuffer() { while (GpsLink(ChAvailable())&&!gps_msg_received) parse_ubx(GpsLink(Getch())); }
#define GpsUartSend1(c) GpsLink(Transmit(c))
#endif /** !SITL */


#endif /* GPS_H */
