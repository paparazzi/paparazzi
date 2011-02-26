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

/** Includes macros generated from ubx.xml */
#include "ubx_protocol.h"

#define GPS_NB_CHANNELS 16

#define GPS_UBX_MAX_PAYLOAD 255
struct GpsUbx {
  bool_t msg_available;
  uint8_t msg_buf[GPS_UBX_MAX_PAYLOAD] __attribute__ ((aligned));
  uint8_t msg_id;
  uint8_t msg_class;

  uint8_t nb_channels;
  uint8_t status;
  uint16_t len;
  uint8_t msg_idx;
  uint8_t ck_a, ck_b;
  uint8_t error_cnt;
  uint8_t error_last;
};

extern struct GpsUbx gps_ubx;

extern void gps_ubx_read_message(void);
extern void gps_ubx_parse(uint8_t c);

#define GpsEvent(_sol_available_callback) {     \
    if (GpsBuffer()) {							\
      ReadGpsBuffer();							\
    }                                           \
    if (gps_ubx.msg_available) {                \
      gps_ubx_read_message();					\
      if (gps_ubx.msg_class == UBX_NAV_ID &&    \
          gps_ubx.msg_id == UBX_NAV_SOL_ID) {   \
        if (gps.fix == GPS_FIX_3D) {            \
          gps.lost_counter = 0;                 \
          gps.last_msg_time = cpu_time_sec;     \
        }                                       \
        _sol_available_callback();              \
      }                                         \
      gps_ubx.msg_available = FALSE;            \
    }                                           \
  }

#define ReadGpsBuffer() {					\
    while (GpsLink(ChAvailable())&&!gps_ubx.msg_available)	\
      gps_ubx_parse(GpsLink(Getch()));			\
  }

#endif /* GPS_UBX_H */
