/*
 * Copyright (C) 2014 Sergey Krukowski <softsr@yahoo.de>
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
 * @file subsystems/gps/gps_sim_hitl.c
 * GPS subsystem simulation from rotorcrafts horizontal/vertical reference system
 */

#include "subsystems/gps.h"

bool_t gps_available;
uint32_t gps_sim_hitl_timer;

void gps_impl_init(void) {
  gps.fix = GPS_FIX_NONE;
  gps_available = FALSE;
}
