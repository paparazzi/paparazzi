/*
 * Copyright (C) Freek van Tienen <freek.v.tienen@gmail.com>
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
 * @file arch/chibios/modules/sensors/hx711.h
 * Interface for the HX711 sensor
 *
 */

#include "std.h"

#ifndef HX711_H
#define HX711_H

extern float hx711_ground_threshold;
extern int32_t hx711_offset;

void hx711_init(void);
void hx711_event(void);
extern bool hx711_ground_detect(void);
extern void hx711_autoset_offset(int32_t offset);

#endif /* HX711_H */