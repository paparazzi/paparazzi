/*
 * Copyright (C) 2025 Jean-Baptiste FORESTIER <jean-baptiste.forestier@enac.fr>
 *
 * This file is part of paparazzi
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
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */
/**
 * @file "modules/gps/gps_nmea_send.h"
 * @author Jean-Baptiste FORESTIER
 * @brief module used to send GPS data over a Tawaki UART for extern instrument using NMEA protocol
 * Exemple of use : MAPIR camera stores GPS data in metadata on each frame
 */

#ifndef GPS_NMEA_SEND_H
#define GPS_NMEA_SEND_H

#include "std.h"

struct gps_nmea_send_msg_t {
  double lat;         ///< Latitude
  double lon;         ///< Longiitude
  uint8_t num_sv;     ///< number of sat in fix
  uint16_t pdop;      ///< position dilution of precision scaled by 100
  float hmsl;         ///< Orthometric height (MSL reference)
  float vground;      ///< Speed over ground in m/s
  float course;       ///< GPS course over ground in rad*1e7, [0, 2*Pi]*1e7 (CW/north)
};

struct Gps_Nmea_Send {
  bool error_init;    // Flag to indicate if there was an error during initialization
  struct gps_nmea_send_msg_t msg;
};

extern void gps_nmea_send_init(void);
extern void gps_nmea_send_periodic(void);

#endif
