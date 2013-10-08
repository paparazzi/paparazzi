/*
 * Copyright (C) 2010 Martin Mueller
 * Copyright (C) 2013 Felix Ruess <felix.ruess@gmail.com>
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

/**
 * @file modules/sensors/baro_bmp.h
 * Bosch BMP085 I2C sensor interface.
 *
 * This reads the values for pressure and temperature from the Bosch BMP085 sensor through I2C.
 */

#ifndef BARO_BMP_H
#define BARO_BMP_H

#include "peripherals/bmp085.h"

extern struct Bmp085 baro_bmp;

/// new measurement every 3rd baro_bmp_periodic
#define BARO_BMP_DT (BARO_BMP_PERIODIC_PERIOD / 3)

extern bool_t baro_bmp_enabled;
extern float baro_bmp_r;
extern float baro_bmp_sigma2;
extern int32_t baro_bmp_alt;

void baro_bmp_init(void);
void baro_bmp_periodic(void);
void baro_bmp_event(void);

#endif
