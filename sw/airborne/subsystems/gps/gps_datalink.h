/*
 * Copyright (C) 2014 Freek van Tienen
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

/**
 * @file gps_datalink.h
 * @brief GPS system based on datalink
 *
 * This GPS parses the datalink REMOTE_GPS packet and sets the
 * GPS structure to the values received.
 */

#ifndef GPS_DATALINK_H
#define GPS_DATALINK_H

#include "std.h"
#include "generated/airframe.h"
#define GPS_NB_CHANNELS 0

extern bool_t gps_available;
#ifdef GPS_USE_DATALINK_SMALL
void parse_gps_datalink_small(uint8_t num_sv, uint32_t pos_xyz, uint32_t speed_xy);
#endif
extern void parse_gps_datalink(uint8_t numsv, int32_t ecef_x, int32_t ecef_y, int32_t ecef_z,
                               int32_t lat, int32_t lon, int32_t alt, int32_t hmsl,
                               int32_t ecef_xd, int32_t ecef_yd, int32_t ecef_zd,
                               uint32_t tow, int32_t course);

// dummy
#define GpsEvent() {}

#endif /* GPS_DATALINK_H */
