#ifndef NPS_SENSOR_GPS_H
#define NPS_SENSOR_GPS_H

#include "pprz_algebra.h"
#include "pprz_algebra_double.h"
#include "pprz_algebra_float.h"
#include "pprz_geodetic_double.h"

#include "std.h"

struct NpsSensorGps {
  struct EcefCoor_d ecef_pos;
  struct EcefCoor_d ecef_vel;
  double       next_update;
  bool_t       data_available;
};


extern void nps_sensor_gps_init(struct NpsSensorGps* gps, double time);
extern void nps_sensor_gps_run_step(struct NpsSensorGps* gps, double time);

#endif /* NPS_SENSOR_GPS_H */
