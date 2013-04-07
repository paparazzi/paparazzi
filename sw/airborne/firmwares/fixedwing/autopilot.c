/*
 * Copyright (C) 2012 Gautier Hattenberger
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
 * @file firmwares/fixedwing/autopilot.c
 *
 * Fixedwing autopilot inititalization.
 *
 */

#include "firmwares/fixedwing/autopilot.h"

uint8_t pprz_mode;
bool_t kill_throttle;

bool_t launch;

/** flight time in seconds. */
uint16_t autopilot_flight_time;

uint8_t lateral_mode;

uint16_t vsupply;
float energy;

bool_t gps_lost;

bool_t power_switch;

void autopilot_init(void) {
  pprz_mode = PPRZ_MODE_AUTO2;
  kill_throttle = FALSE;
  launch = FALSE;
  autopilot_flight_time = 0;

  lateral_mode = LATERAL_MODE_MANUAL;

  gps_lost = FALSE;

  power_switch = FALSE;

}

