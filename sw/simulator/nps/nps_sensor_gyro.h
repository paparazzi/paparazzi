#ifndef NPS_SENSOR_GYRO_H
#define NPS_SENSOR_GYRO_H

#include "math/pprz_algebra.h"
#include "math/pprz_algebra_double.h"
#include "math/pprz_algebra_float.h"
#include "std.h"

struct NpsSensorGyro {
  struct DoubleVect3  value;
  int min;
  int max;
  struct DoubleMat33  sensitivity;
  struct DoubleVect3  neutral;
  struct DoubleVect3  noise_std_dev;
  struct DoubleVect3  bias_initial;
  struct DoubleVect3  bias_random_walk_std_dev;
  struct DoubleVect3  bias_random_walk_value;
  double       next_update;
  bool_t       data_available;
};


extern void   nps_sensor_gyro_init(struct NpsSensorGyro *gyro, double time);
extern void   nps_sensor_gyro_run_step(struct NpsSensorGyro *gyro, double time, struct DoubleRMat *body_to_imu);

#endif /* NPS_SENSOR_GYRO_H */

