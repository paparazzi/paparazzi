#include "booz_sensors_model.h"

#include <math.h>

#include "booz_flight_model.h"


struct BoozSensorsModel bsm;

void booz_sensors_model_init(void) {
  bsm.accel = v_get(AXIS_NB);
  bsm.accel->ve[AXIS_X] = 0.;
  bsm.accel->ve[AXIS_Y] = 0.;
  bsm.accel->ve[AXIS_Z] = 0.;


  bsm.gyro = v_get(AXIS_NB);
  bsm.gyro->ve[AXIS_P] = 0.;
  bsm.gyro->ve[AXIS_Q] = 0.;
  bsm.gyro->ve[AXIS_R] = 0.;

  bsm.gyro_sensitivity = m_get(AXIS_NB, AXIS_NB);
  m_zero(bsm.gyro_sensitivity);
  bsm.gyro_sensitivity->me[AXIS_P][AXIS_P] = 1./-0.0002202;
  bsm.gyro_sensitivity->me[AXIS_Q][AXIS_Q] = 1./-0.0002150;
  bsm.gyro_sensitivity->me[AXIS_R][AXIS_R] = 1./ 0.0002104;

  bsm.gyro_neutral = v_get(AXIS_NB);
  bsm.gyro_neutral->ve[AXIS_P] = 40885;
  bsm.gyro_neutral->ve[AXIS_Q] = 40910;
  bsm.gyro_neutral->ve[AXIS_R] = 39552;

}


void booz_sensors_model_run(void) {
  
  /* extract rotational speed from flight model state */
  static VEC *rate_body = VNULL;
  rate_body = v_resize(rate_body, AXIS_NB);
  rate_body->ve[AXIS_P] = bfm.state->ve[BFMS_P];
  rate_body->ve[AXIS_Q] = bfm.state->ve[BFMS_Q];
  rate_body->ve[AXIS_R] = bfm.state->ve[BFMS_R];

  /* compute sensor readings */
  bsm.gyro = mv_mlt(bsm.gyro_sensitivity, rate_body, bsm.gyro); 
  bsm.gyro = v_add(bsm.gyro, bsm.gyro_neutral, bsm.gyro); 
  bsm.gyro->ve[AXIS_P] = round( bsm.gyro->ve[AXIS_P]);
  bsm.gyro->ve[AXIS_Q] = round( bsm.gyro->ve[AXIS_Q]);
  bsm.gyro->ve[AXIS_R] = round( bsm.gyro->ve[AXIS_R]);

}
