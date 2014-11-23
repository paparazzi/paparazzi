/*
 * Copyright (C) 2004-2011 The Paparazzi Team
 *               2014 Freek van Tienen <freek.v.tienen@gmail.com>
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

/** @file gps_nmea.h
 * NMEA protocol specific code.
 *
*/


#ifndef GPS_NMEA_H
#define GPS_NMEA_H

#include "mcu_periph/uart.h"

#ifndef DEBUG_NMEA
#define NMEA_PRINT(...) {};
#endif

#define NMEA_MAXLEN 255

struct GpsNmea {
  bool_t msg_available;
  bool_t pos_available;
  uint8_t gps_nb_ovrn;        // number if incomplete nmea-messages
  char msg_buf[NMEA_MAXLEN];  ///< buffer for storing one nmea-line
  int msg_len;
};

extern struct GpsNmea gps_nmea;


/*
 * This part is used by the autopilot to read data from a uart
 */
#define __GpsLink(dev, _x) dev##_x
#define _GpsLink(dev, _x)  __GpsLink(dev, _x)
#define GpsLink(_x) _GpsLink(GPS_LINK, _x)

#define GpsBuffer() GpsLink(ChAvailable())

#define GpsEvent(_sol_available_callback) {             \
    nmea_parse_prop_init();                             \
    if (GpsBuffer()) {                                  \
      ReadGpsBuffer();                                  \
    }                                                   \
    if (gps_nmea.msg_available) {                       \
      gps.last_msg_ticks = sys_time.nb_sec_rem;         \
      gps.last_msg_time = sys_time.nb_sec;              \
      nmea_parse_msg();                                 \
      if (gps_nmea.pos_available) {                     \
        if (gps.fix == GPS_FIX_3D) {                    \
          gps.last_3dfix_ticks = sys_time.nb_sec_rem;   \
          gps.last_3dfix_time = sys_time.nb_sec;        \
        }                                               \
        _sol_available_callback();                      \
      }                                                 \
      gps_nmea.msg_available = FALSE;                   \
    }                                                   \
  }

#define ReadGpsBuffer() {         \
    while (GpsLink(ChAvailable())&&!gps_nmea.msg_available) \
      nmea_parse_char(GpsLink(Getch()));      \
  }


/** The function to be called when a characted friom the device is available */
extern void nmea_parse_char(uint8_t c);
extern void nmea_parse_msg(void);
extern uint8_t nmea_calc_crc(const char *buff, int buff_sz);
extern void nmea_parse_prop_init(void);
extern void nmea_parse_prop_msg(void);

/** Read until a certain character, placed here for proprietary includes */
static inline void nmea_read_until(int *i)
{
  while (gps_nmea.msg_buf[(*i)++] != ',') {
    if (*i >= gps_nmea.msg_len) {
      return;
    }
  }
}

#endif /* GPS_NMEA_H */
