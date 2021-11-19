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
#include "modules/gps/gps.h"

#ifndef PRIMARY_GPS
#define PRIMARY_GPS GPS_DATALINK
#endif

extern struct GpsState gps_datalink;

extern void gps_datalink_init(void);

#define gps_datalink_periodic_check() gps_periodic_check(&gps_datalink)

extern void gps_datalink_parse_REMOTE_GPS(uint8_t *buf);
extern void gps_datalink_parse_REMOTE_GPS_SMALL(uint8_t *buf);
extern void gps_datalink_parse_REMOTE_GPS_LOCAL(uint8_t *buf);

#endif /* GPS_DATALINK_H */
