#ifndef BOOZ_SENSORS_MODEL_UTILS_H
#define BOOZ_SENSORS_MODEL_UTILS_H

#include <matrix.h>
#include <glib.h>

struct BoozDatedSensor {
  VEC* value;
  double time;
};

extern void UpdateSensorLatency(double time, VEC* cur_reading, GSList **history,
			 double latency, VEC* sensor_reading);
extern VEC* v_update_random_walk(VEC* in, VEC* std_dev, double dt, VEC* out);
extern VEC* v_add_gaussian_noise(VEC* in, VEC* std_dev, VEC* out);
extern VEC* v_get_gaussian_noise(VEC* std_dev, VEC* out);
extern double get_gaussian_noise();

#define RoundSensor(_sensor) {				\
    _sensor->ve[AXIS_X] = rint(_sensor->ve[AXIS_X]);	\
    _sensor->ve[AXIS_Y] = rint(_sensor->ve[AXIS_Y]);	\
    _sensor->ve[AXIS_Z] = rint(_sensor->ve[AXIS_Z]);	\
  }

#define BoundSensor(_sensor, _min, _max) {		       \
  if ( _sensor->ve[AXIS_X] < _min) _sensor->ve[AXIS_X] = _min; \
  if ( _sensor->ve[AXIS_X] > _max) _sensor->ve[AXIS_X] = _max; \
  if ( _sensor->ve[AXIS_Y] < _min) _sensor->ve[AXIS_Y] = _min; \
  if ( _sensor->ve[AXIS_Y] > _max) _sensor->ve[AXIS_Y] = _max; \
  if ( _sensor->ve[AXIS_Z] < _min) _sensor->ve[AXIS_Z] = _min; \
  if ( _sensor->ve[AXIS_Z] > _max) _sensor->ve[AXIS_Z] = _max; \
  }



#endif /* BOOZ_SENSORS_MODEL_UTILS_H */
