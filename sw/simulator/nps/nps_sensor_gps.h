#ifndef NPS_SENSOR_GPS_H
#define NPS_SENSOR_GPS_H

#include <glib.h>

#include "math/pprz_algebra.h"
#include "math/pprz_algebra_double.h"
#include "math/pprz_algebra_float.h"
#include "math/pprz_geodetic_double.h"

#include "std.h"

struct NpsSensorGps {
  struct EcefCoor_d ecef_pos;
  struct EcefCoor_d ecef_vel;
  struct LlaCoor_d  lla_pos;
  double hmsl;
  struct DoubleVect3  pos_noise_std_dev;
  struct DoubleVect3  speed_noise_std_dev;
  struct DoubleVect3  pos_bias_initial;
  struct DoubleVect3  pos_bias_random_walk_std_dev;
  struct DoubleVect3  pos_bias_random_walk_value;
  double pos_latency;
  double speed_latency;
  GSList *hmsl_history;
  GSList *pos_history;
  GSList *lla_history;
  GSList *speed_history;
  double next_update;
  bool_t data_available;
};


extern void nps_sensor_gps_init(struct NpsSensorGps *gps, double time);
extern void nps_sensor_gps_run_step(struct NpsSensorGps *gps, double time);

#endif /* NPS_SENSOR_GPS_H */
