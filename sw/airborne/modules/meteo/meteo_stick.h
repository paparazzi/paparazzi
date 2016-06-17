/*
 * Copyright (C) 2014 Gautier Hattenberger
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
 * @file modules/meteo/meteo_stick.h
 *
 * Data acquisition module for ENAC PTU board
 *
 * provides meteo data:
 * - temperature
 * - humidity
 * - absolute pressure
 * - differential pressure
 *
 * based on the ADS1220 spi ADC for pressures and temperature,
 * and PWM input for humidity
 *
 * data can be send other telemetry
 * and/or logged on SD card (on capable autopilots)
 *
 * designed and debugged by Michel Gorraz and Alexandre Bustico @ Enac (2014)
 *
 */

#ifndef METEO_STICK_H
#define METEO_STICK_H

#include "std.h"
#include "peripherals/ads1220.h"

#ifndef USE_MS_EEPROM
#define USE_MS_EEPROM TRUE
#endif

#if USE_MS_EEPROM
#include "peripherals/eeprom25AA256.h"
#include "modules/meteo/meteo_stick_calib.h"
#endif

/** Raw sensors structure */
struct MeteoStick {
  struct Ads1220 pressure;      ///< absolute pressure
  struct Ads1220 diff_pressure; ///< differential pressure
  struct Ads1220 temperature;   ///< temperature
  uint32_t humidity_period;     ///< humidity (in ticks)
  float current_temperature;    ///< calibrated temperature
  float current_pressure;       ///< calibrated pressure
  float current_humidity;       ///< calibrated humidity
  float current_airspeed;       ///< calibrated airspeed
#if USE_MS_EEPROM
  struct Eeprom25AA256 eeprom;  ///< eeprom with calibration data
  Calibration_params calib;     ///< calibration
#endif
  bool reset_dp_offset;       ///< reset differential pressure offset
};

extern struct MeteoStick meteo_stick;

/** Functions */
extern void meteo_stick_init(void);
extern void meteo_stick_periodic(void);
extern void meteo_stick_event(void);

#endif

