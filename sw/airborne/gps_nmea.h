/*
 * Paparazzi autopilot $Id: gps_ubx.h 6376 2010-11-07 04:30:44Z flixr $
 *
 * Copyright (C) 2004-2006  Pascal Brisset, Antoine Drouin
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

/** \file gps_nmea.h
 * \brief NMEA protocol specific code
 *
*/


#ifndef GPS_NMEA_H
#define GPS_NMEA_H

#define GPS_NB_CHANNELS 16
#include "gps.h"

#ifdef DEBUG_NMEA
#define NMEA_PRINT(...) {  UsbSPrintString( __VA_ARGS__);};
#else
#define NMEA_PRINT(...) {};
#endif

void parse_nmea_GPGSA(void);
void parse_nmea_GPRMC(void);
void parse_nmea_GPGGA(void);

extern uint16_t gps_reset;



/** The function to be called when a characted friom the device is available */
void parse_nmea_char( uint8_t c );


#define GpsParse(_gps_buffer, _gps_buffer_size) { \
  uint8_t i; \
  for(i = 0; i < _gps_buffer_size; i++) { \
    parse_ubx(_gps_buffer[i]); \
  } \
}

#define GpsFixValid() (gps_mode == 3)

#define gps_nmea_Reset(_val) { \
  gps_reset = _val; \
}

#ifdef GPS_TIMESTAMP
uint32_t itow_from_ticks(uint32_t clock_ticks);
#endif
#define ubxsend_cfg_rst(a,b) {};
#endif /* GPS_NMEA_H */
