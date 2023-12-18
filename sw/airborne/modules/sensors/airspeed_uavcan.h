/*
 * Copyright (C) 2023 Freek van Tienen <freek.v.tienen@gmail.com>
 *
 * This file is part of Paparazzi.
 *
 * Paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * Paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 */

/** @file modules/sensors/airspeed_uavcan.h
 * Airspeed sensor on the uavcan bus
 */

#ifndef AIRSPEED_UAVCAN_H
#define AIRSPEED_UAVCAN_H

#include "std.h"

/* Airspeed UAVCAN structure */
struct airspeed_uavcan_t {
  float diff_p;       ///< Differential pressure
  float temperature;  ///< Temperature in Celsius

  float diff_p_offset;  ///< Differential pressure offset
  float diff_p_scale;   ///< Differential pressure scale
};
extern struct airspeed_uavcan_t airspeed_uavcan;

/* External functions */
extern void airspeed_uavcan_init(void);
extern void airspeed_uavcan_autoset_offset(bool set);

#endif /* AIRSPEED_UAVCAN_H */