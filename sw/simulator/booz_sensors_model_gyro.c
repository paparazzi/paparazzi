#include "booz_sensors_model.h"

#include "booz_sensors_model_params.h"
#include "booz_sensors_model_utils.h"
#include "booz_flight_model.h"

bool_t booz_sensors_model_gyro_available() {
  if (bsm.gyro_available) {
    bsm.gyro_available = FALSE;
    return TRUE;
  }
  return FALSE;
}

void booz_sensors_model_gyro_init(double time) {

  bsm.gyro = v_get(AXIS_NB);
  bsm.gyro->ve[AXIS_P] = 0.;
  bsm.gyro->ve[AXIS_Q] = 0.;
  bsm.gyro->ve[AXIS_R] = 0.;
  bsm.gyro_resolution = BSM_GYRO_RESOLUTION;

  bsm.gyro_sensitivity = m_get(AXIS_NB, AXIS_NB);
  m_zero(bsm.gyro_sensitivity);
  bsm.gyro_sensitivity->me[AXIS_P][AXIS_P] = BSM_GYRO_SENSITIVITY_PP;
  bsm.gyro_sensitivity->me[AXIS_Q][AXIS_Q] = BSM_GYRO_SENSITIVITY_QQ;
  bsm.gyro_sensitivity->me[AXIS_R][AXIS_R] = BSM_GYRO_SENSITIVITY_RR;

  bsm.gyro_neutral = v_get(AXIS_NB);
  bsm.gyro_neutral->ve[AXIS_P] = BSM_GYRO_NEUTRAL_P;
  bsm.gyro_neutral->ve[AXIS_Q] = BSM_GYRO_NEUTRAL_Q;
  bsm.gyro_neutral->ve[AXIS_R] = BSM_GYRO_NEUTRAL_R;

  bsm.gyro_noise_std_dev = v_get(AXIS_NB);
  bsm.gyro_noise_std_dev->ve[AXIS_P] = BSM_GYRO_NOISE_STD_DEV_P;
  bsm.gyro_noise_std_dev->ve[AXIS_Q] = BSM_GYRO_NOISE_STD_DEV_Q;
  bsm.gyro_noise_std_dev->ve[AXIS_R] = BSM_GYRO_NOISE_STD_DEV_R;

  bsm.gyro_bias_initial = v_get(AXIS_NB);
  bsm.gyro_bias_initial->ve[AXIS_P] = BSM_GYRO_BIAS_INITIAL_P;
  bsm.gyro_bias_initial->ve[AXIS_Q] = BSM_GYRO_BIAS_INITIAL_Q;
  bsm.gyro_bias_initial->ve[AXIS_R] = BSM_GYRO_BIAS_INITIAL_R;

  bsm.gyro_bias_random_walk_std_dev = v_get(AXIS_NB);
  bsm.gyro_bias_random_walk_std_dev->ve[AXIS_P] = BSM_GYRO_BIAS_RANDOM_WALK_STD_DEV_P;
  bsm.gyro_bias_random_walk_std_dev->ve[AXIS_Q] = BSM_GYRO_BIAS_RANDOM_WALK_STD_DEV_Q;
  bsm.gyro_bias_random_walk_std_dev->ve[AXIS_R] = BSM_GYRO_BIAS_RANDOM_WALK_STD_DEV_R;

  bsm.gyro_bias_random_walk_value = v_get(AXIS_NB);
  bsm.gyro_bias_random_walk_value->ve[AXIS_P] = bsm.gyro_bias_initial->ve[AXIS_P];
  bsm.gyro_bias_random_walk_value->ve[AXIS_Q] = bsm.gyro_bias_initial->ve[AXIS_Q];
  bsm.gyro_bias_random_walk_value->ve[AXIS_R] = bsm.gyro_bias_initial->ve[AXIS_R];

  bsm.gyro_next_update = time;
  bsm.gyro_available = FALSE;

}

void booz_sensors_model_gyro_run( double time ) {
  if (time < bsm.gyro_next_update)
    return;

  /* extract rotational speed from flight model state */
  static VEC *rate_body = VNULL;
  rate_body = v_resize(rate_body, AXIS_NB);
  rate_body->ve[AXIS_P] = bfm.state->ve[BFMS_P];
  rate_body->ve[AXIS_Q] = bfm.state->ve[BFMS_Q];
  rate_body->ve[AXIS_R] = bfm.state->ve[BFMS_R];

  /* compute gyros readings */
  bsm.gyro = mv_mlt(bsm.gyro_sensitivity, rate_body, bsm.gyro); 
  bsm.gyro = v_add(bsm.gyro, bsm.gyro_neutral, bsm.gyro); 

  /* compute gyro error readings */
  static VEC *gyro_error = VNULL;
  gyro_error = v_resize(gyro_error, AXIS_NB);
  gyro_error = v_zero(gyro_error);
  /* add a gaussian noise */
  gyro_error = v_add_gaussian_noise(gyro_error, bsm.gyro_noise_std_dev, gyro_error);
  /* update random walk bias */
  bsm.gyro_bias_random_walk_value = 
    v_update_random_walk(bsm.gyro_bias_random_walk_value, 
  			 bsm.gyro_bias_random_walk_std_dev, BSM_GYRO_DT, 
  			 bsm.gyro_bias_random_walk_value);
  gyro_error = v_add(gyro_error, bsm.gyro_bias_random_walk_value, gyro_error); 
  /* scale to adc units FIXME : should use full adc gain ? sum ? */
  gyro_error->ve[AXIS_P] = 
    gyro_error->ve[AXIS_P] * bsm.gyro_sensitivity->me[AXIS_P][AXIS_P];
  gyro_error->ve[AXIS_Q] = 
    gyro_error->ve[AXIS_Q] * bsm.gyro_sensitivity->me[AXIS_Q][AXIS_Q];
  gyro_error->ve[AXIS_R] = 
    gyro_error->ve[AXIS_R] * bsm.gyro_sensitivity->me[AXIS_R][AXIS_R];
  /* add per gyro error reading */
  bsm.gyro =  v_add(bsm.gyro, gyro_error, bsm.gyro); 
  /* round signal to account for adc discretisation */
  RoundSensor(bsm.gyro);
  /* saturation                                     */
  BoundSensor(bsm.gyro, 0, bsm.gyro_resolution); 

  bsm.gyro_next_update += BSM_GYRO_DT;
  bsm.gyro_available = TRUE;
}


