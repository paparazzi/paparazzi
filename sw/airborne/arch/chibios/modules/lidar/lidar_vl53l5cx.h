/*
 * Copyright (C) 2024 Fabien-B <name.surname@gmail.com>
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

/** @file "modules/lidar/lidar_vl53l5cx.h"
 * @author Fabien-B <name.surname@gmail.com>
 * VL53L5CX multizone range sensor.
 */

#ifndef LIDAR_VL53L5CX_H
#define LIDAR_VL53L5CX_H

#include "stdint.h"
#include "peripherals/vl53l5cx_api.h"

extern void lidar_vl53l5cx_init(void);
extern void lidar_vl53l5cx_periodic(void);

#endif  // LIDAR_VL53L5CX_H
