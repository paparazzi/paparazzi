/*
 * $Id$
 *  
 * Copyright (C) 2008 Antoine Drouin
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

#ifndef BOOZ_SENSORS_MODEL_H
#define BOOZ_SENSORS_MODEL_H

#include <matrix.h>
#include <glib.h>

#include "6dof.h"
#include "std.h"

extern void booz_sensors_model_init(double time);
extern void booz_sensors_model_run( double time);
extern bool_t booz_sensors_model_accel_available();
extern bool_t booz_sensors_model_gyro_available();
extern bool_t booz_sensors_model_mag_available();
extern bool_t booz_sensors_model_baro_available();
extern bool_t booz_sensors_model_gps_available();


struct BoozSensorsModel {

  /* Accelerometer */
  VEC*   accel;
  unsigned int accel_resolution;
  MAT*   accel_sensitivity;
  VEC*   accel_neutral;
  VEC*   accel_noise_std_dev;
  VEC*   accel_bias;
  double accel_next_update;
  int    accel_available;


  /* Gyrometer */
  VEC*   gyro;
  unsigned int gyro_resolution;
  MAT*   gyro_sensitivity;
  VEC*   gyro_neutral;
  VEC*   gyro_noise_std_dev;
  VEC*   gyro_bias_initial;
  VEC*   gyro_bias_random_walk_std_dev;
  VEC*   gyro_bias_random_walk_value;
  double gyro_next_update;
  int    gyro_available;


  /* Magnetometer */
  VEC*   mag;
  unsigned int mag_resolution;
  MAT*   mag_sensitivity;
  VEC*   mag_neutral;
  VEC*   mag_noise_std_dev;
  double mag_next_update;
  int    mag_available;

  
  /* Rangemeter */
  double rangemeter;
  double rangemeter_next_update;
  int    rangemeter_available;
  

  /* Barometer */
  double baro;
  unsigned int baro_resolution;
  double baro_next_update;
  int    baro_available;


  /* GPS */
  VEC*    gps_speed;
  double  gps_speed_course;
  double  gps_speed_gspeed;
  double  gps_speed_climb;
  VEC*    gps_speed_noise_std_dev;
  GSList* gps_speed_history;
  VEC*    gps_pos;
  double  gps_pos_utm_north;
  double  gps_pos_utm_east;
  double  gps_pos_utm_alt;
  VEC*    gps_pos_noise_std_dev;
  VEC*    gps_pos_bias_initial;
  VEC*    gps_pos_bias_random_walk_std_dev;
  VEC*    gps_pos_bias_random_walk_value;
  GSList* gps_pos_history;
  double  gps_next_update;
  int     gps_available;

};

extern struct BoozSensorsModel bsm;


#endif /* BOOZ_SENSORS_MODEL_H */
