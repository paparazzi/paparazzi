#ifndef NPS_SENSORS_H
#define NPS_SENSORS_H

#include "nps_sensor_gyro.h"
//nclude "nps_sensor_accel.h"
//nclude "nps_sensor_mag.h"
//nclude "nps_sensor_baro.h"
//#include "nps_sensor_gps.h"


struct NpsSensors {

  struct NpsSensorGyro  gyro;
  //  struct NpsSensorAccel accel;
  //  struct NpsSensorMag   mag;
  //  struct NpsSensorBaro  baro;
  //  struct NpsSensorGps   gps;

};

extern struct NpsSensors sensors;

#endif /* NPS_SENSORS_H */
