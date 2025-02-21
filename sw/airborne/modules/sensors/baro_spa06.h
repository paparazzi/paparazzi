/*
 * Florian Sansou florian.sansou@enac.fr
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
 *
 */

/**
 * @file modules/sensors/baro_spa06.h
 *  SPA06  sensor interface.
 *
 * This reads the values for pressure and temperature from the  SPA06 sensor.
 */

#ifndef BARO_SPA06_H
#define BARO_SPA06_H

#include "peripherals/spa06.h"

extern struct spa06_t baro_spa06;

extern float baro_alt;
extern  bool baro_alt_valid;

void baro_spa06_init(void);
void baro_spa06_periodic(void);
void baro_spa06_event(void);

#endif
