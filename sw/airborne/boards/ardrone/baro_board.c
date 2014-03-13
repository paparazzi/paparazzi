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
#include "subsystems/abi.h"
#include "baro_board.h"
#include "navdata.h"

/** Use an extra median filter to filter baro data
 */
#if USE_BARO_MEDIAN_FILTER
#include "filters/median_filter.h"
struct MedianFilterInt baro_median;
#endif


#define BMP180_OSS 0  // Parrot ARDrone uses no oversampling

void baro_init(void) {
#if USE_BARO_MEDIAN_FILTER
  init_median_filter(&baro_median);
#endif
}

void baro_periodic(void) {}

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
  int32_t press = p + ((x1 + x2 + 3791L) >> 4);
  // Zero at sealevel
  return press;
}

static inline int32_t baro_apply_calibration_temp(int32_t tmp_raw)
{
  int32_t x1 = ((tmp_raw - ((int32_t)baro_calibration.ac6)) * ((int32_t)baro_calibration.ac5)) >> 15;
  int32_t x2 = (((int32_t)baro_calibration.mc) << 11) / (x1 + ((int32_t)baro_calibration.md));
  baro_calibration.b5 = x1 + x2;
  return (baro_calibration.b5 + 8) >> 4;
}

void ardrone_baro_event(void)
{
  if (navdata_baro_available) {
    if (baro_calibrated) {
      // first read temperature because pressure calibration depends on temperature
      // TODO send Temperature message
      baro_apply_calibration_temp(navdata.temperature_pressure);
      int32_t press_raw = baro_apply_calibration(navdata.pressure);
#if USE_BARO_MEDIAN_FILTER
      press_raw = update_median_filter(&baro_median, press_raw);
#endif
      float pressure = (float)press_raw;
      AbiSendMsgBARO_ABS(BARO_BOARD_SENDER_ID, &pressure);
    }
    navdata_baro_available = FALSE;
  }
}
