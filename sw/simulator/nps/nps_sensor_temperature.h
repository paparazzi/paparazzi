#ifndef NPS_SENSOR_TEMPERATURE_H
#define NPS_SENSOR_TEMPERATURE_H

#include "math/pprz_algebra.h"
#include "math/pprz_algebra_double.h"
#include "math/pprz_algebra_float.h"
#include "std.h"

struct NpsSensorTemperature {
  double  value;          ///< temperature in degrees Celcius
  double  noise_std_dev;  ///< noise standard deviation
  double  next_update;
  bool  data_available;
};


extern void nps_sensor_temperature_init(struct NpsSensorTemperature *temperature, double time);
extern void nps_sensor_temperature_run_step(struct NpsSensorTemperature *temperature, double time);

#endif /* NPS_SENSOR_TEMPERATURE_H */
