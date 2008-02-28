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
  VEC*    gps_speed_noise_std_dev;
  GSList* gps_speed_history;
  VEC*    gps_pos;
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
