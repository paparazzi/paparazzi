#ifndef NPS_SENSOR_BARO_H
#define NPS_SENSOR_BARO_H

#include "math/pprz_algebra.h"
#include "math/pprz_algebra_double.h"
#include "math/pprz_algebra_float.h"
#include "std.h"

struct NpsSensorBaro {
  double  value;          ///< pressure in Pascal
  double  noise_std_dev;  ///< noise standard deviation
  double  next_update;
  bool  data_available;
};


extern void nps_sensor_baro_init(struct NpsSensorBaro *baro, double time);
extern void nps_sensor_baro_run_step(struct NpsSensorBaro *baro, double time);

#endif /* NPS_SENSOR_BARO_H */
