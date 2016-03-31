#ifndef NPS_SENSOR_MAG_H
#define NPS_SENSOR_MAG_H

#include "math/pprz_algebra.h"
#include "math/pprz_algebra_double.h"
#include "math/pprz_algebra_float.h"
#include "std.h"

struct NpsSensorMag {
  struct DoubleVect3  value;
  int min;
  int max;
  struct DoubleMat33 sensitivity;
  struct DoubleVect3 neutral;
  struct DoubleVect3 noise_std_dev;
  struct DoubleRMat  imu_to_sensor_rmat;
  double       next_update;
  bool       data_available;
};


extern void nps_sensor_mag_init(struct NpsSensorMag *mag, double time);
extern void nps_sensor_mag_run_step(struct NpsSensorMag *mag, double time, struct DoubleRMat *body_to_imu);

#endif /* NPS_SENSOR_MAG_H */
