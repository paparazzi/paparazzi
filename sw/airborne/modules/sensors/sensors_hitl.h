/*
 * Copyright (C) 2023 Gautier Hattenberger <gautier.hattenberger@enac.fr>
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

#ifndef SENSORS_HITL_H
#define SENSORS_HITL_H

#include "std.h"

#ifndef PRIMARY_GPS
#define PRIMARY_GPS GPS_SIM
#endif

extern bool gps_has_fix;

extern void sensors_hitl_init(void);
extern void sensors_hitl_periodic(void);
extern void sensors_hitl_event(void);
extern void sensors_hitl_parse_HITL_IMU(uint8_t *buf);
extern void sensors_hitl_parse_HITL_GPS(uint8_t *buf);
extern void sensors_hitl_parse_HITL_AIR_DATA(uint8_t *buf);

// dummy definition for compilation
extern void imu_feed_gyro_accel(void);
extern void imu_feed_mag(void);
extern void gps_feed_value(void);

#endif /* SENSORS_HITL_H */

