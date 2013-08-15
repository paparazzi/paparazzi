/*
 * Copyright (C) 2013 TU Delft Quatrotor Team 1
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

/**
 * @file boards/ardrone/baro_board.c
 * Paparazzi AR Drone 2 Baro Sensor implementation:.
 *
 * Based on BMP180 implementation by C. de Wagter.
 */

#include "subsystems/sensors/baro.h"
#include "baro_board.h"
#include "navdata.h"

struct Baro baro;

// Over sample setting (0-3)
#define BMP180_OSS 0

int32_t pressure;
int32_t temperature;
int32_t pres_raw = 0;
int32_t tmp_raw  = 0;
int32_t pr0      = 0;

void baro_init(void) {
  baro.status = BS_UNINITIALIZED;
  baro.absolute       = 0;
  baro.differential   = 0;
  baro_data_available = 0;
}

static inline int32_t baro_apply_calibration(int32_t raw)
{
  int32_t b6 = ((int32_t)baro_calibration.b5) - 4000L;
  int32_t x1 = (((int32_t)baro_calibration.b2) * (b6 * b6 >> 12)) >> 11;
  int32_t x2 = ((int32_t)baro_calibration.ac2) * b6 >> 11;
  int32_t x3 = x1 + x2;
  int32_t b3 = (((((int32_t)baro_calibration.ac1) * 4 + x3) << BMP180_OSS) + 2)/4;
  x1 = ((int32_t)baro_calibration.ac3) * b6 >> 13;
  x2 = (((int32_t)baro_calibration.b1) * (b6 * b6 >> 12)) >> 16;
  x3 = ((x1 + x2) + 2) >> 2;
  uint32_t b4 = (((int32_t)baro_calibration.ac4) * (uint32_t) (x3 + 32768L)) >> 15;
  uint32_t b7 = (raw - b3) * (50000L >> BMP180_OSS);
  int32_t p = b7 < 0x80000000L ? (b7 * 2) / b4 : (b7 / b4) * 2;
  x1 = (p >> 8) * (p >> 8);
  x1 = (x1 * 3038UL) >> 16;
  x2 = (-7357L * p) >> 16;
  return p + ((x1 + x2 + 3791L) >> 4);
}

static inline void baro_read_temp(void)
{
  tmp_raw = (navdata->temperature_pressure & 0x00FF) << 8 | (navdata->temperature_pressure >> 8);
  int32_t x1 = ((tmp_raw - ((int32_t)baro_calibration.ac6)) * ((int32_t)baro_calibration.ac5)) >> 15;
  int32_t x2 = (((int32_t)baro_calibration.mc) << 11) / (x1 + ((int32_t)baro_calibration.md));
  baro_calibration.b5 = x1 + x2;
  temperature = (baro_calibration.b5 + 8) >> 4;
}

static inline void baro_read_pressure(void)
{
  pres_raw = navdata->pressure >> 8;
  pres_raw = pres_raw >> (8 - BMP180_OSS);
  pressure = baro_apply_calibration(pres_raw);
  if ((pr0 != 0) && (pressure != 0))
    pr0 = pressure;
  pressure -= pr0;
}

void baro_periodic(void) {
  if(baro.status == BS_RUNNING && navdata_baro_available == 1) {
    navdata_baro_available = 0;

    baro_read_temp(); // first read temperature because pressure calibration depends on temperature
    baro_read_pressure();
    baro.absolute = pressure;
    baro_data_available = TRUE;
  }
  else {
    baro_data_available = FALSE;
    if (baro_calibrated == TRUE) {
      baro.status = BS_RUNNING;
    }
  }
}
