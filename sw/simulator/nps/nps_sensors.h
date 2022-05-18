#ifndef NPS_SENSORS_H
#define NPS_SENSORS_H

#include "math/pprz_algebra.h"

#ifndef NPS_SENSORS_PARAMS
#include "nps_sensors_params_default.h"
#else
#include NPS_SENSORS_PARAMS
#endif /* NPS_SENSORS_PARAMS */
#include "nps_sensor_gyro.h"
#include "nps_sensor_accel.h"
#include "nps_sensor_mag.h"
#include "nps_sensor_baro.h"
#include "nps_sensor_gps.h"
#include "nps_sensor_sonar.h"
#include "nps_sensor_airspeed.h"
#include "nps_sensor_temperature.h"
#include "nps_sensor_aoa.h"
#include "nps_sensor_sideslip.h"

struct NpsSensors {
  struct DoubleRMat body_to_imu_rmat;
  struct NpsSensorGyro  gyro;
  struct NpsSensorAccel accel;
  struct NpsSensorMag   mag;
  struct NpsSensorBaro  baro;
  struct NpsSensorGps   gps;
  struct NpsSensorSonar sonar;
  struct NpsSensorAirspeed airspeed;
  struct NpsSensorTemperature temp;
  struct NpsSensorAngleOfAttack aoa;
  struct NpsSensorSideSlip sideslip;
};

extern struct NpsSensors sensors;

extern void nps_sensors_init(double time);
extern void nps_sensors_run_step(double time);

extern bool nps_sensors_gyro_available();
extern bool nps_sensors_mag_available();
extern bool nps_sensors_baro_available();
extern bool nps_sensors_gps_available();
extern bool nps_sensors_sonar_available();
extern bool nps_sensors_airspeed_available();
extern bool nps_sensors_temperature_available();
extern bool nps_sensors_aoa_available();
extern bool nps_sensors_sideslip_available();

#endif /* NPS_SENSORS_H */
