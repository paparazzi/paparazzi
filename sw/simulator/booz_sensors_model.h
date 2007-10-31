#ifndef BOOZ_SENSORS_MODEL_H
#define BOOZ_SENSORS_MODEL_H

#include "6dof.h"
#include <matrix.h>

extern void booz_sensors_model_init(void);
extern void booz_sensors_model_run( double dt);

struct BoozSensorsModel {

  VEC* accel;
  unsigned int accel_resolution;
  MAT* accel_sensitivity;
  VEC* accel_neutral;
  VEC* accel_noise_std_dev;
  VEC* accel_bias;

  VEC* gyro;
  unsigned int gyro_resolution;
  MAT* gyro_sensitivity;
  VEC* gyro_neutral;
  VEC* gyro_noise_std_dev;
  VEC* gyro_bias_initial;
  VEC* gyro_bias_random_walk_std_dev;
  VEC* gyro_bias_random_walk_value;

  VEC* mag;
  unsigned int mag_resolution;
  MAT* mag_sensitivity;
  VEC* mag_neutral;
  VEC* mag_noise_std_dev;


  /* imaginary sensors - gps maybe */
  VEC* speed_sensor;
  VEC* speed_noise_std_dev;
  VEC* pos_sensor;
  VEC* pos_noise_std_dev;
  VEC* pos_bias_initial;
  VEC* pos_bias_random_walk_std_dev;
  VEC* pos_bias_random_walk_value;

};

extern struct BoozSensorsModel bsm;

#endif /* BOOZ_SENSORS_MODEL_H */
