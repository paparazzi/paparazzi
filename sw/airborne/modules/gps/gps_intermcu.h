/*
 * Copyright (C) 2022 Gautier Hattenberger <gautier.hattenberger@enac.fr>
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
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */

/**
 * @file modules/gps/gps_intermcu.h
 * @brief GPS system based on intermcu
 */

#ifndef GPS_INTERMCU_H
#define GPS_INTERMCU_H

#include "std.h"
#include "generated/airframe.h"
#include "modules/gps/gps.h"

#ifndef PRIMARY_GPS
#define PRIMARY_GPS GPS_INTERMCU
#endif

extern struct GpsState gps_imcu;

extern void gps_intermcu_init(void);

#define gps_intermcu_periodic_check() gps_periodic_check(&gps_imcu)

extern void gps_intermcu_parse_IMCU_REMOTE_GPS(uint8_t *buf);

#endif /* GPS_INTERMCU_H */
