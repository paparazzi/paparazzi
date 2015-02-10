/*
 * Copyright (C) 2015 Gautier Hattenberger, Alexandre Bustico
 *
 * This file is part of paparazzi

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
 * @file modules/meteo/meteo_stick_calib.h
 *
 * Parser for calibration data coming from 25AA256 EEPROM
 * of the Meteo Stick
 *
 */

#ifndef METEO_STICK_CALIB_H
#define METEO_STICK_CALIB_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <time.h>

#define  MTOSTK_MAX_SEEK_ARRAY_SIZE 20 // max polynom order is 4
#define  MTOSTK_MAX_POLY_ARRAY_SIZE 6 // max polynom order is 4
#define  MTOSTK_MAX_TEMP_ARRAY_SIZE 6 //
#define  UUID_LEN 13 // including trailing 0

typedef enum {
  MTOSTK_TEMP = 0,
  MTOSTK_ABS_PRESS,
  MTOSTK_DIF_PRESS,
  MTOSTK_HUMIDITY,
  MTOSTK_NUM_SENSORS
} Mtostk_sensors ;

/**
 * @struct Sensors_params
 * @brief  Calibration_params structure initialized with calibvration data read from meteostick eeprom
 *
 * @var  coeffs     two dimensions array for polynomial coefficient for each calibration temperature (one or more)
 * @var  temps      one dimension array for temperature of calibration
 * @var  timestamp  date of calibration in time_t unix time
 * @var  num_temp   number of temerature of calibration, at least 1
 * @var  num_coeff  number of coefficient for the polynom. the polynom order is num_coeff-1
 */
typedef struct {
  float  coeffs[MTOSTK_MAX_TEMP_ARRAY_SIZE][MTOSTK_MAX_POLY_ARRAY_SIZE];
  float  temps[MTOSTK_MAX_TEMP_ARRAY_SIZE];
  time_t timestamp;
  uint16_t num_temp;
  uint16_t num_coeff;
} Sensors_params;

typedef struct {
  Sensors_params params[MTOSTK_NUM_SENSORS];
  char uuid[13];
} Calibration_params;


/**
 * @brief   Initializes a Calibration_params structure from memory buffer
 * @pre     the memory buffer must be initialized with data read from the spi eeprom embeded on the meteostick
 *
 * @param[out] cp pointer to the Calibration_params structure to be initialized
 * @param[in]  eeprom_buffer pointer to word aligned buffer of bytes
 */
extern bool mtostk_populate_cal_from_buffer(Calibration_params *cp, uint8_t *eeprom_buffer);

/**
 * @brief get calibrated value for a sensor according to current temperature
 *
 * @param[in] cp pointer to calibration structure
 * @param[in] type sensor type
 * @param[in] uncal uncalibrated value
 * @param[in] temp current temperature (not relevant for calibration of temperature sensor)
 * @return calibrated value
 */
extern float mtostk_get_calibrated_value(Calibration_params *cp, Mtostk_sensors type, float uncal, float temp);

#endif

