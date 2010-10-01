#ifndef LIBEKNAV_RAW_LOG_H
#define LIBEKNAV_RAW_LOG_H

#include "math/pprz_algebra_float.h"

struct raw_log_entry {
  float time;
  struct FloatRates   gyro;
  struct FloatVect3   accel;
  struct FloatVect3   mag;
  struct FloatVect3		ecef_pos;
  struct FloatVect3		ecef_vel;
};

#endif /* LIBEKNAV_RAW_LOG_H */


