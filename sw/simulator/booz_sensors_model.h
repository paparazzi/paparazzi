#ifndef BOOZ_SENSORS_MODEL_H
#define BOOZ_SENSORS_MODEL_H

#include "6dof.h"
#include <matrix.h>

extern void booz_sensors_model_init(void);
extern void booz_sensors_model_run(void);

struct BoozSensorsModel {
  VEC* accel;
  VEC* gyro;
  MAT* gyro_sensitivity;
  VEC* gyro_neutral;
};

extern struct BoozSensorsModel bsm;

#endif /* BOOZ_SENSORS_MODEL_H */
