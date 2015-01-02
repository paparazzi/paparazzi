#ifndef LIBEKNAV_RAW_LOG_H
#define LIBEKNAV_RAW_LOG_H

//#include "math/pprz_algebra_float.h"
#include "fms/fms_autopilot_msg.h"

/*
struct __attribute__ ((packed)) raw_log_entry{
  float time;
  struct FloatRates   gyro;
  struct FloatVect3   accel;
  struct FloatVect3   mag;
  struct FloatVect3   ecef_pos;
  struct FloatVect3   ecef_vel;
  int16_t pressure_absolute;
  uint8_t data_valid;
};
*/
struct __attribute__((packed)) raw_log_entry {
  float time;
  struct AutopilotMessageVIUp message;
};

#endif /* LIBEKNAV_RAW_LOG_H */


