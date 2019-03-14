/*
 * Copyright (C) 2019 Gautier Hattenberger <gautier.hattenberger@enac.fr>
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
 * @file modules/sensors/baro_bmp3.h
 * Bosch BMP3 I2C sensor interface.
 *
 * This reads the values for pressure and temperature from the Bosch BMP3 sensor through I2C.
 */

#ifndef BARO_BMP3_H
#define BARO_BMP3_H

#include "peripherals/bmp3_i2c.h"

extern struct Bmp3_I2c baro_bmp3;

void baro_bmp3_init(void);
void baro_bmp3_periodic(void);
void baro_bmp3_event(void);

#endif
