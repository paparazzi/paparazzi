/*
 * Copyright (C) 2020 Tom van Dijk <tomvand@users.noreply.github.com>
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

#ifndef SONAR_VL53L1X_H
#define SONAR_VL53L1X_H

#include "peripherals/vl53l1x_api.h"

struct sonar_vl53l1x_dev {
  VL53L1_Dev_t dev;
  int16_t offset_mm;
  uint8_t read_state;
};
extern struct sonar_vl53l1x_dev sonar_vl53l1x;

extern void sonar_vl53l1x_init(void);
extern void sonar_vl53l1x_read(void);

#endif
