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
#include "modules/gps/gps.h"

#define GPS_NMEA_NB_CHANNELS 12

#define NMEA_MAXLEN 255

#ifndef PRIMARY_GPS
#define PRIMARY_GPS GPS_NMEA
#endif

struct GpsNmea {
  bool msg_available;       ///< flag set to TRUE if a new msg/sentence is available to be parsed
  bool is_configured;       ///< flag set to TRUE if configuration is finished
  bool have_gsv;            ///< flag set to TRUE if GPGSV message received
  uint8_t gps_nb_ovrn;        ///< number if incomplete nmea-messages
  char msg_buf[NMEA_MAXLEN];  ///< buffer for storing one nmea-line
  int msg_len;
  uint8_t status;             ///< line parser status

  struct GpsState state;
};

extern struct GpsNmea gps_nmea;

extern void gps_nmea_init(void);
extern void gps_nmea_event(void);

#define gps_nmea_periodic_check() gps_periodic_check(&gps_nmea.state)


/*
 * This part is used by the autopilot to read data from a uart
 */

/** The function to be called when a characted from the device is available */
#include "pprzlink/pprzlink_device.h"

extern void nmea_configure(void);
extern void nmea_parse_char(uint8_t c);
extern bool nmea_parse_msg(void);
extern uint8_t nmea_calc_crc(const char *buff, int buff_sz);
extern void nmea_parse_prop_init(void);
extern bool nmea_parse_prop_msg(void);
extern void nmea_gps_msg(void);

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
