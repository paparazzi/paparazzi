#ifndef NPS_SENSORS_GYRO_H
#define NPS_SENSORS_GYRO_H

#include "pprz_algebra_double.h"

struct NpsSensorGyro {
  struct DoubleVect3  value;
  unsigned int resolution;
  struct DoubleMat33  sensitivity;
  struct DoubleVect3  neutral;
  struct DoubleVect3  noise_std_dev;
  struct DoubleVect3  bias_initial;
  struct DoubleVect3  bias_random_walk_std_dev;
  struct DoubleVect3  bias_random_walk_value;
  double       next_update;
  int          available;
};

#endif /* NPS_SENSORS_GYRO_H */

