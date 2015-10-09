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
 *
 */

/** @file drop_zone.c
 *  @brief Drop zone
 */

#include "multi/drop_zone.h"
#include "generated/flight_plan.h"

uint8_t drop_zone_nb = 1;

void drop_zone_set(uint8_t dz) {
  drop_zone_nb = dz;

  if(dz == 3) {
    waypoint_copy(WP_DROP, WP_RZ3);
  } else if(dz == 2) {
    waypoint_copy(WP_DROP, WP_RZ2);
  } else {
    waypoint_copy(WP_DROP, WP_RZ1);
  }
}
