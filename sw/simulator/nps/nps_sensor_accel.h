#ifndef NPS_SENSOR_ACCEL_H
#define NPS_SENSOR_ACCEL_H

#include "math/pprz_algebra.h"
#include "math/pprz_algebra_double.h"
#include "math/pprz_algebra_float.h"
#include "std.h"

struct NpsSensorAccel {
  struct DoubleVect3  value;
  int min;
  int max;
  struct DoubleMat33  sensitivity;
  struct DoubleVect3  neutral;
  struct DoubleVect3  noise_std_dev;
  struct DoubleVect3  bias;
  double       next_update;
  bool       data_available;
};


extern void   nps_sensor_accel_init(struct NpsSensorAccel *accel, double time);
extern void   nps_sensor_accel_run_step(struct NpsSensorAccel *accel, double time, struct DoubleRMat *body_to_imu);

#endif /* NPS_SENSOR_ACCEL_H */
