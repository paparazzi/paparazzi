#ifndef NPS_SENSORS_H
#define NPS_SENSORS_H

#include "nps_sensors_gyro.h"

struct NpsSensors {

  struct NpsSensorsGyro gyro;
  struct NpsSensorsGyro accel;

};


#endif /* NPS_SENSORS_H */
