/*
 * Copyright (C) 2019 Gautier Hattenberger <gautier.hattenberger@enac.fr>
 *                    Titouan Verdu <titouan.verdu@enac.fr>
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
 */

/**
 * @file "modules/meteo/cloud_sensor.h"
 *
 * Get data from Cloud Sensor
 * - compute coef value from PAYLOAD_FLOAT data
 *   - Liquid Water Content (LWC)
 *   - Angstrom coef
 *   - single sensor
 * - get already computed LWC from PAYLOAD_COMMAND data
 */

#ifndef CLOUD_SENSOR_H
#define CLOUD_SENSOR_H

#include "std.h"

#define CLOUD_SENSOR_COEF_NONE      0
#define CLOUD_SENSOR_COEF_SINGLE    1
#define CLOUD_SENSOR_COEF_ANGSTROM  2

/**
 * variables for settings
 */
extern uint8_t cloud_sensor_compute_coef;
extern uint8_t cloud_sensor_compute_background;
extern float cloud_sensor_threshold;
extern float cloud_sensor_hysteresis;
extern float cloud_sensor_background;
extern float cloud_sensor_calib_alpha;
extern float cloud_sensor_calib_beta;
extern float cloud_sensor_channel_scale;
extern float cloud_sensor_tau;
extern void cloud_sensor_update_tau(float tau); // setting handler

/** Init function
 */
extern void cloud_sensor_init(void);

/** New message/data callback
 */
extern void cloud_sensor_callback(uint8_t *buf);
extern void LWC_callback(uint8_t *buf);

/** Report function
 */
extern void cloud_sensor_report(void);


#endif

