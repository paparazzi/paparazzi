/*
 * Copyright (C) 2012 Freek van Tienen
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

/**
 * @file gps_sirf.h
 * @brief Sirf protocol specific code
 *
 */

#ifndef GPS_SIRF_H
#define GPS_SIRF_H

#include "std.h"
#include "subsystems/gps.h"

#ifndef PRIMARY_GPS
#define PRIMARY_GPS gps_sirf
#endif

#define SIRF_GPS_NB_CHANNELS 16
#define SIRF_MAXLEN 255

//Read states
#define UNINIT  0
#define GOT_A0  1
#define GOT_A2  2
#define GOT_B0  3

struct GpsSirf {
  bool msg_available;
  bool msg_valid;
  char msg_buf[SIRF_MAXLEN];  ///< buffer for storing one nmea-line
  int msg_len;
  int read_state;
  struct GpsState state;
};

extern struct GpsSirf gps_sirf;

extern void gps_sirf_init(void);
extern void gps_sirf_event(void);
extern void gps_sirf_register(void);


#endif /* GPS_SIRF_H */
