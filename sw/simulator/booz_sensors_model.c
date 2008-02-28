#include "booz_sensors_model.h"
#include "booz_sensors_model_params.h"

#include <math.h>

#include "std.h"
#include "booz_flight_model.h"
#include "booz_flight_model_utils.h"

struct BoozSensorsModel bsm;

extern void booz_sensors_model_accel_init(double time);
extern void booz_sensors_model_accel_run(double time, MAT* dcm);

extern void booz_sensors_model_gyro_init(double time);
extern void booz_sensors_model_gyro_run(double time);

extern void booz_sensors_model_mag_init(double time);
extern void booz_sensors_model_mag_run(double time, MAT* dcm);

extern void booz_sensors_model_rangemeter_init(double time);
extern void booz_sensors_model_rangemeter_run(double time);

extern void booz_sensors_model_baro_init(double time);
extern void booz_sensors_model_baro_run(double time);

extern void booz_sensors_model_gps_init(double time);
extern void booz_sensors_model_gps_run(double time, MAT* dcm_t);


void booz_sensors_model_init(double time) {
  booz_sensors_model_accel_init(time);
  booz_sensors_model_gyro_init(time);
  booz_sensors_model_mag_init(time);
  booz_sensors_model_rangemeter_init(time);
  booz_sensors_model_baro_init(time);
  booz_sensors_model_gps_init(time);
} 

void booz_sensors_model_run(double time) {

  /* extract eulers angles from state */
  static VEC *eulers = VNULL;
  eulers = v_resize(eulers, AXIS_NB);
  eulers->ve[EULER_PHI]   =  bfm.state->ve[BFMS_PHI];
  eulers->ve[EULER_THETA] =  bfm.state->ve[BFMS_THETA];
  eulers->ve[EULER_PSI]   =  bfm.state->ve[BFMS_PSI];
  /* direct cosine matrix ( inertial to body )*/
  static MAT *dcm = MNULL;
  dcm = m_resize(dcm,AXIS_NB, AXIS_NB);
  dcm = dcm_of_eulers(eulers, dcm);
  /* transpose of dcm ( body to inertial ) */
  static MAT *dcm_t = MNULL;
  dcm_t = m_resize(dcm_t,AXIS_NB, AXIS_NB);
  dcm_t = m_transp(dcm, dcm_t);

  booz_sensors_model_accel_run(time, dcm);
  booz_sensors_model_gyro_run(time);
  booz_sensors_model_mag_run(time, dcm);
  booz_sensors_model_rangemeter_run(time);
  booz_sensors_model_baro_run(time);
  booz_sensors_model_gps_run(time, dcm_t);
} 






