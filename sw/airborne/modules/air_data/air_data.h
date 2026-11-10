/*
 * Copyright (C) 2013 Gautier Hattenberger
 *               2014 Felix Ruess <felix.ruess@gmail.com>
 *
 * This file is part of paparazzi
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
 */

/**
 * @file modules/air_data/air_data.h
 * Air Data interface
 *  - pressures (several barometers and differential pressure sensors)
 *  - airspeed
 *  - angle of attack and sideslip
 *  - wind
 */

#ifndef AIR_DATA_H
#define AIR_DATA_H

#include "std.h"
#include "pprzlink/pprzlink_device.h"
#include "pprzlink/pprzlink_transport.h"

/** Maximum number of sensors of each kind (barometers, differential pressure, airspeed, temperature) */
#ifndef AIR_DATA_MAX_SENSORS
#define AIR_DATA_MAX_SENSORS 4
#endif

/** Calibration bitmask of a sensor */
struct air_data_calib_t {
  bool offset: 1;   ///< Offset calibrated (airframe or calibration)
  bool scale: 1;    ///< Scale calibrated (airframe)
};

/** One air data sensor: barometer, differential pressure, airspeed or temperature.
 * The drivers send the raw values (BARO_ABS_RAW, BARO_DIFF_RAW, AIRSPEED_RAW, TEMPERATURE_RAW)
 * and the calibrated value is (raw - offset) * scale.
 */
struct air_data_sensor_t {
  uint8_t abi_id;                     ///< ABI sender id of the driver, ABI_DISABLE when unused
  uint8_t type;                       ///< Temperature type (TEMPERATURE_TYPE_*), only for the temperatures
  struct air_data_calib_t calibrated; ///< Calibration bitmask
  float offset;                       ///< Offset, in the raw unit
  float scale;                        ///< Scale
  float raw;                          ///< Last raw value (Pa, m/s or deg Celcius)
  float value;                        ///< Last calibrated value (Pa, m/s or deg Celcius)
  uint32_t stamp;                     ///< Driver timestamp of the last value (us)
  float time;                         ///< System time of the last valid value (s), 0 if never
  float calib_sum;                    ///< Sum of the raw values while calibrating
  uint16_t calib_cnt;                 ///< Number of raw values while calibrating
};

/** Air Data structure */
struct AirData {
  struct air_data_sensor_t baros[AIR_DATA_MAX_SENSORS];     ///< Barometers (static pressure)
  struct air_data_sensor_t diffs[AIR_DATA_MAX_SENSORS];     ///< Differential pressure sensors
  struct air_data_sensor_t airspeeds[AIR_DATA_MAX_SENSORS]; ///< Sensors that only give an airspeed
  struct air_data_sensor_t temps[AIR_DATA_MAX_SENSORS];     ///< Temperatures

  uint8_t baro_id;      ///< Barometer to use (ABI id), ABI_BROADCAST for the first healthy one
  uint8_t airspeed_id;  ///< Differential pressure or airspeed sensor to use (ABI id), ABI_BROADCAST for automatic
  uint8_t temp_id;      ///< Temperature to use (ABI id), ABI_BROADCAST for automatic (air, then pitot, then baro)
  uint8_t baro_used;    ///< Barometer in use (ABI id), ABI_DISABLE if none
  uint8_t airspeed_used;///< Airspeed sensor in use (ABI id), ABI_DISABLE if none
  uint8_t temp_used;    ///< Temperature in use (ABI id), ABI_DISABLE if none

  bool calibrate;       ///< Calibrate all sensors (zero the differential pressures, line up the barometers), true while running
  bool calib_armed;     ///< Allow the calibration while the motors are armed
  float qfe;            ///< Static pressure at the last calibration (Pa), -1 if unknown

  float pressure;     ///< Static atmospheric pressure (Pa), -1 if unknown
  float differential; ///< Differential pressure (total - static pressure) (Pa)
  float temperature;  ///< temperature in degrees Celcius, -1000 if unknown

  float airspeed;     ///< Equivalent Air Speed (equals to Calibrated Air Speed at low speed/altitude) (in m/s, -1 if unknown
  float tas;          ///< True Air Speed (TAS) in m/s, -1 if unknown
  float tas_factor;   ///< factor to convert equivalent airspeed (EAS) to true airspeed (TAS), from pressure and temperature (or ISA at the altitude)
  float qnh;              ///< Barometric pressure adjusted to sea level in hPa, -1 if unknown
  float amsl_baro;        ///< altitude above sea level in m from pressure and QNH
  bool amsl_baro_valid; ///< TRUE if #amsl_baro is currently valid
  bool calc_qnh_once;   ///< flag to calculate QNH with next pressure measurement

  float aoa;          ///< angle of attack (rad)
  float sideslip;     ///< sideslip angle (rad)
  float wind_speed;   ///< wind speed (m/s)
  float wind_dir;     ///< wind direction (rad, 0 north, >0 clockwise)
};

/** global AirData state
 */
extern struct AirData air_data;

/** AirData initialization. Called at startup.
 */
extern void air_data_init(void);

/** Check health. Needs to be called periodically.
 */
extern void air_data_periodic(void);

/** Start the calibration of all sensors (setting handler).
 * Zeroes the differential pressures and lines up the barometers with the one in use.
 * Refused while the motors are armed, unless #AirData.calib_armed.
 */
extern void air_data_Calibrate(bool calibrate);

/** Find (or create) the sensor of an ABI id.
 * @param sensors The sensors (air_data.baros, diffs, airspeeds or temps)
 * @param abi_id The ABI sender id
 * @param create Take a free slot if not found
 * @return The sensor, NULL if not found or full
 */
extern struct air_data_sensor_t *air_data_get_sensor(struct air_data_sensor_t *sensors, uint8_t abi_id, bool create);

/** Parse datalink wind info message
 */
extern void air_data_parse_WIND_INFO(struct link_device *dev, struct transport_tx *trans, uint8_t *buf);

/** Return AMSL (altitude AboveSeaLevel).
 * If AMSL from baro is valid, return that, otherwise from gps.
 */
extern float air_data_get_amsl(void);

/**
 * Calculate equivalent airspeed from dynamic pressure.
 * Dynamic pressure @f$q@f$ (also called impact pressure) is the
 * difference between total(pitot) and static pressure.
 *
 * @param q dynamic pressure in Pa
 * @return equivalent airspeed in m/s
 */
extern float eas_from_dynamic_pressure(float q);

/**
 * Calculate true airspeed (TAS) factor.
 * TAS = tas_factor * EAS
 *
 * @param p current air pressure in Pa
 * @param t current air temperature in degrees Celcius
 * @return tas factor
 */
extern float get_tas_factor(float p, float t);

/**
 * Calculate true airspeed from equivalent airspeed.
 *
 * @param eas equivalent airspeed (EAS) in m/s
 * @return true airspeed in m/s
 */
extern float tas_from_eas(float eas);

/**
 * Calculate equivalent airspeed from true airspeed.
 *
 * @param tas true airspeed (TAS) in m/s
 * @return equivalent airspeed in m/s
 */
extern float eas_from_tas(float tas);

/**
 * Calculate true airspeed from dynamic pressure.
 * Dynamic pressure @f$q@f$ (also called impact pressure) is the
 * difference between total(pitot) and static pressure.
 *
 * @param q dynamic pressure in Pa
 * @return true airspeed in m/s
 */
extern float tas_from_dynamic_pressure(float q);


#endif
