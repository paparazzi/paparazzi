/*
 * Copyright (C) 2011 Martin Mueller <martinmm@pfump.org>
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
 * @file modules/sensors/baro_ms5611_spi.h
 * Measurement Specialties (Intersema) MS5611-01BA pressure/temperature sensor interface for SPI.
 *
 */

#ifndef BARO_MS56111_SPI_H
#define BARO_MS56111_SPI_H

#include "std.h"
#include "peripherals/ms5611_spi.h"

/// new measurement with every baro_ms5611_read() call
#define BARO_MS5611_DT BARO_MS5611_READ_PERIOD
#define BARO_MS5611_R 20
#define BARO_MS5611_SIGMA2 1
extern float baro_ms5611_r;
extern float baro_ms5611_sigma2;

extern float baro_ms5611_alt;
extern bool_t baro_ms5611_alt_valid;
extern bool_t baro_ms5611_enabled;

extern struct Ms5611_Spi baro_ms5611;

extern void baro_ms5611_init(void);
extern void baro_ms5611_read(void);
extern void baro_ms5611_periodic_check(void);
extern void baro_ms5611_event(void);
extern void baro_ms5611_send_coeff(void);

#define BaroMs5611UpdatePressure(_b, _h) { if (baro_ms5611.data_available) { _b = baro_ms5611.data.pressure; _h(); baro_ms5611.data_available = FALSE; } }

#define BaroMs5611UpdateAlt(_b, _h) { if (baro_ms5611.data_available) { _b = baro_ms5611_alt; _h(); baro_ms5611.data_available = FALSE; } }

#endif
