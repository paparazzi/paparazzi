/*
 * Copyright (C) Freek van Tienen <freek.v.tienen@gmail.com>
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
 * @file "modules/sensors/rpm_sensor.h"
 * @author Freek van Tienen <freek.v.tienen@gmail.com>
 * RPM sensor based on time difference between pulses
 */

#ifndef RPM_SENSOR_H
#define RPM_SENSOR_H

#include "std.h"

/* Default pulses per round */
#ifndef RPM_PULSE_PER_RND
#define RPM_PULSE_PER_RND 14
#endif

extern void rpm_sensor_init(void);
extern void rpm_sensor_periodic(void);
extern uint16_t rpm_sensor_get_rpm(void);

#endif

