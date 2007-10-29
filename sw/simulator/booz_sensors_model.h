#ifndef BOOZ_SENSORS_MODEL_H
#define BOOZ_SENSORS_MODEL_H

#include "6dof.h"
#include <matrix.h>

extern void booz_sensors_model_init(void);
extern void booz_sensors_model_run(void);

struct BoozSensorsModel {

  VEC* accel;
  MAT* accel_sensitivity;
  VEC* accel_neutral;
  VEC* accel_noise_std_dev;
  VEC* accel_bias;

  VEC* gyro;
  MAT* gyro_sensitivity;
  VEC* gyro_neutral;
  VEC* gyro_noise_std_dev;
  VEC* gyro_bias;

  /* imaginary sensors - gps maybe */
  VEC* speed_sensor;
  VEC* pos_sensor;

};

extern struct BoozSensorsModel bsm;

#endif /* BOOZ_SENSORS_MODEL_H */
