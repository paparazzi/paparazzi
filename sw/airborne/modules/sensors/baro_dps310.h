/*
 * Copyright (C) 2026 OpenUAS
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
 * @file modules/sensors/baro_dps310.h
 * Infineon DPS310 I2C sensor interface.
 *
 * This code reads the values for pressure and temperature from the Infineon DPS310 sensor over I2C.
 */

#ifndef BARO_DPS310_H
#define BARO_DPS310_H

#include "peripherals/dps310_i2c.h"

extern struct Dps310_I2c baro_dps310;

void baro_dps310_init(void);
void baro_dps310_periodic(void);
void baro_dps310_event(void);

#endif
