#include "booz_sensors_model.h"

#include BSM_PARAMS
#include "booz_sensors_model_utils.h"
#include "booz_flight_model.h"
#include "booz_flight_model_utils.h"


bool_t booz_sensors_model_accel_available() {
  if (bsm.accel_available) {
    bsm.accel_available = FALSE;
    return TRUE;
  }
  return FALSE;
}


void booz_sensors_model_accel_init(double time) {

  bsm.accel = v_get(AXIS_NB);
  bsm.accel->ve[AXIS_X] = 0.;
  bsm.accel->ve[AXIS_Y] = 0.;
  bsm.accel->ve[AXIS_Z] = 0.;
  bsm.accel_resolution = BSM_ACCEL_RESOLUTION;

  bsm.accel_sensitivity = m_get(AXIS_NB, AXIS_NB);
  m_zero(bsm.accel_sensitivity);
  bsm.accel_sensitivity->me[AXIS_X][AXIS_X] = BSM_ACCEL_SENSITIVITY_XX;
  bsm.accel_sensitivity->me[AXIS_Y][AXIS_Y] = BSM_ACCEL_SENSITIVITY_YY;
  bsm.accel_sensitivity->me[AXIS_Z][AXIS_Z] = BSM_ACCEL_SENSITIVITY_ZZ;

  bsm.accel_neutral = v_get(AXIS_NB);
  bsm.accel_neutral->ve[AXIS_X] = BSM_ACCEL_NEUTRAL_X;
  bsm.accel_neutral->ve[AXIS_Y] = BSM_ACCEL_NEUTRAL_Y;
  bsm.accel_neutral->ve[AXIS_Z] = BSM_ACCEL_NEUTRAL_Z;

  bsm.accel_noise_std_dev = v_get(AXIS_NB);
  bsm.accel_noise_std_dev->ve[AXIS_X] = BSM_ACCEL_NOISE_STD_DEV_X;
  bsm.accel_noise_std_dev->ve[AXIS_Y] = BSM_ACCEL_NOISE_STD_DEV_Y;
  bsm.accel_noise_std_dev->ve[AXIS_Z] = BSM_ACCEL_NOISE_STD_DEV_Z;

  bsm.accel_bias = v_get(AXIS_NB);
  bsm.accel_bias->ve[AXIS_P] = BSM_ACCEL_BIAS_X;
  bsm.accel_bias->ve[AXIS_Q] = BSM_ACCEL_BIAS_Y;
  bsm.accel_bias->ve[AXIS_R] = BSM_ACCEL_BIAS_Z;

  bsm.accel_next_update = time;
  bsm.accel_available = FALSE;

}

void booz_sensors_model_accel_run( double time ) {
  if (time < bsm.accel_next_update)
    return;

  static VEC* accel_ltp = VNULL;
  accel_ltp = v_resize(accel_ltp, AXIS_NB);
  /* substract gravity to acceleration in ltp frame */
  accel_ltp = v_sub(bfm.accel_ltp, bfm.g_ltp, accel_ltp);
  /* convert to body frame */
  static VEC* accel_body = VNULL;
  accel_body = v_resize(accel_body, AXIS_NB);
  mv_mlt(bfm.dcm, accel_ltp, accel_body);
  /* convert to imu frame */
  static VEC* accel_imu = VNULL;
  accel_imu = v_resize(accel_imu, AXIS_NB);
  mv_mlt(bsm.body_to_imu, accel_body, accel_imu);



  //  printf(" accel_body ~ %f %f %f\n", accel_body->ve[AXIS_X], accel_body->ve[AXIS_Y], accel_body->ve[AXIS_Z]);

  /* compute accel reading */
  mv_mlt(bsm.accel_sensitivity, accel_imu, bsm.accel);
  v_add(bsm.accel, bsm.accel_neutral, bsm.accel);

  /* compute accel error readings */
  static VEC *accel_error = VNULL;
  accel_error = v_resize(accel_error, AXIS_NB);
  accel_error = v_zero(accel_error);
  /* add a gaussian noise */
  accel_error = v_add_gaussian_noise(accel_error, bsm.accel_noise_std_dev, accel_error);
  /* constant bias  */
  accel_error = v_add(accel_error, bsm.accel_bias, accel_error);
  /* scale to adc units FIXME : should use full adc gain ? sum ? */
  accel_error->ve[AXIS_X] = accel_error->ve[AXIS_X] * bsm.accel_sensitivity->me[AXIS_X][AXIS_X];
  accel_error->ve[AXIS_Y] = accel_error->ve[AXIS_Y] * bsm.accel_sensitivity->me[AXIS_Y][AXIS_Y];
  accel_error->ve[AXIS_Z] = accel_error->ve[AXIS_Z] * bsm.accel_sensitivity->me[AXIS_Z][AXIS_Z];
  /* add per accel error reading */
  bsm.accel =  v_add(bsm.accel, accel_error, bsm.accel);
  /* round signal to account for adc discretisation */
  RoundSensor(bsm.accel);
  /* saturation                                     */
  BoundSensor(bsm.accel, 0, bsm.accel_resolution);

  //  printf("sim adc %f %f %f\n",bsm.accel->ve[AXIS_X] ,bsm.accel->ve[AXIS_Y] ,bsm.accel->ve[AXIS_Z]);
  bsm.accel_next_update += BSM_ACCEL_DT;
  bsm.accel_available = TRUE;
}

