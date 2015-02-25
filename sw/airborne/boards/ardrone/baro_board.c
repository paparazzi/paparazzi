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

void baro_init(void)
{
#if USE_BARO_MEDIAN_FILTER
  init_median_filter(&baro_median);
#endif
}

void baro_periodic(void) {}

/**
 * Apply temperature and sensor calibration to get pressure in Pa.
 * @param raw uncompensated raw pressure reading
 * @return compensated pressure in Pascal
 */
static inline int32_t baro_apply_calibration(int32_t raw)
{
  int32_t b6 = ((int32_t)navdata.bmp180_calib.b5) - 4000L;
  int32_t x1 = (((int32_t)navdata.bmp180_calib.b2) * (b6 * b6 >> 12)) >> 11;
  int32_t x2 = ((int32_t)navdata.bmp180_calib.ac2) * b6 >> 11;
  int32_t x3 = x1 + x2;
  int32_t b3 = (((((int32_t)navdata.bmp180_calib.ac1) * 4 + x3) << BMP180_OSS) + 2) / 4;
  x1 = ((int32_t)navdata.bmp180_calib.ac3) * b6 >> 13;
  x2 = (((int32_t)navdata.bmp180_calib.b1) * (b6 * b6 >> 12)) >> 16;
  x3 = ((x1 + x2) + 2) >> 2;
  uint32_t b4 = (((int32_t)navdata.bmp180_calib.ac4) * (uint32_t)(x3 + 32768L)) >> 15;
  uint32_t b7 = (raw - b3) * (50000L >> BMP180_OSS);
  int32_t p = b7 < 0x80000000L ? (b7 * 2) / b4 : (b7 / b4) * 2;
  x1 = (p >> 8) * (p >> 8);
  x1 = (x1 * 3038UL) >> 16;
  x2 = (-7357L * p) >> 16;
  int32_t press = p + ((x1 + x2 + 3791L) >> 4);
  // Zero at sealevel
  return press;
}

/**
 * Apply temperature calibration.
 * @param tmp_raw uncompensated raw temperature reading
 * @return compensated temperature in 0.1 deg Celcius
 */
static inline int32_t baro_apply_calibration_temp(int32_t tmp_raw)
{
  int32_t x1 = ((tmp_raw - ((int32_t)navdata.bmp180_calib.ac6)) * ((int32_t)navdata.bmp180_calib.ac5)) >> 15;
  int32_t x2 = (((int32_t)navdata.bmp180_calib.mc) << 11) / (x1 + ((int32_t)navdata.bmp180_calib.md));
  navdata.bmp180_calib.b5 = x1 + x2;
  return (navdata.bmp180_calib.b5 + 8) >> 4;
}

void ardrone_baro_event(void)
{
  if (navdata.baro_available) {
    if (navdata.baro_calibrated) {
      // first read temperature because pressure calibration depends on temperature
      float temp_deg = 0.1 * baro_apply_calibration_temp(navdata.measure.temperature_pressure);
      AbiSendMsgTEMPERATURE(BARO_BOARD_SENDER_ID, temp_deg);
      int32_t press_pascal = baro_apply_calibration(navdata.measure.pressure);
#if USE_BARO_MEDIAN_FILTER
      press_pascal = update_median_filter(&baro_median, press_pascal);
#endif
      float pressure = (float)press_pascal;
      AbiSendMsgBARO_ABS(BARO_BOARD_SENDER_ID, pressure);
    }
    navdata.baro_available = FALSE;
  }
}
