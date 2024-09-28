/*
 * Copyright (C) 2023 Dennis van Wijngaarden <D.C.vanWijngaarden@tudelft.nl>
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

/** @file "modules/nav/ground_detect_sensor.h"
 * @author Dennis van Wijngaarden <D.C.vanWijngaarden@tudelft.nl>
 * Ground detection module
 */

#ifndef GROUND_DETECT_SENSOR_H
#define GROUND_DETECT_SENSOR_H

#include "std.h"

extern void ground_detect_sensor_init(void);
extern void ground_detect_sensor_periodic(void);

extern bool ground_detect(void);

#endif  // GROUND_DETECT_SENSOR_H
