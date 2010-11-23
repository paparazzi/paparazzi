#include "booz_sensors_model.h"
#include BSM_PARAMS

#include <math.h>

#include "std.h"
#include "booz_flight_model.h"
#include "booz_flight_model_utils.h"

struct BoozSensorsModel bsm;

extern void booz_sensors_model_accel_init(double time);
extern void booz_sensors_model_accel_run(double time);

extern void booz_sensors_model_gyro_init(double time);
extern void booz_sensors_model_gyro_run(double time);

extern void booz_sensors_model_mag_init(double time);
extern void booz_sensors_model_mag_run(double time);

extern void booz_sensors_model_rangemeter_init(double time);
extern void booz_sensors_model_rangemeter_run(double time);

extern void booz_sensors_model_baro_init(double time);
extern void booz_sensors_model_baro_run(double time);

extern void booz_sensors_model_gps_init(double time);
extern void booz_sensors_model_gps_run(double time);


void booz_sensors_model_init(double time) {

  VEC* tmp_eulers = v_get(AXIS_NB);
  tmp_eulers->ve[EULER_PHI]   = BSM_BODY_TO_IMU_PHI;
  tmp_eulers->ve[EULER_THETA] = BSM_BODY_TO_IMU_THETA;
  tmp_eulers->ve[EULER_PSI]   = BSM_BODY_TO_IMU_PSI;
  bsm.body_to_imu = m_get(AXIS_NB, AXIS_NB);
  dcm_of_eulers (tmp_eulers, bsm.body_to_imu );

  booz_sensors_model_accel_init(time);
  booz_sensors_model_gyro_init(time);
  booz_sensors_model_mag_init(time);
  booz_sensors_model_rangemeter_init(time);
  booz_sensors_model_baro_init(time);
  booz_sensors_model_gps_init(time);

}

void booz_sensors_model_run(double time) {

  booz_sensors_model_accel_run(time);
  booz_sensors_model_gyro_run(time);
  booz_sensors_model_mag_run(time);
  booz_sensors_model_rangemeter_run(time);
  booz_sensors_model_baro_run(time);
  booz_sensors_model_gps_run(time);

}






