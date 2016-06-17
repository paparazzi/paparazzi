#ifndef NPS_SENSORS_UTILS_H
#define NPS_SENSORS_UTILS_H

#include <glib.h>
#include "math/pprz_algebra_double.h"

struct BoozDatedSensor {
  struct DoubleVect3 *value;
  double time;
};

struct BoozDatedSensor_Single {
  double *value;
  double time;
};

/* cur_reading and sensor_reading must be of a type that can be cast to DoubleVect3* */
extern void UpdateSensorLatency(double time, gpointer cur_reading, GSList **history,
                                double latency, gpointer sensor_reading);

/* ...and the same for single double values */
extern void UpdateSensorLatency_Single(double time, gpointer cur_reading, GSList **history,
                                       double latency, gpointer sensor_reading);

#endif /* NPS_SENSORS_UTILS_H */
