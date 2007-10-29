#include "booz_sensors_model.h"

#include <math.h>

#include "booz_flight_model.h"


struct BoozSensorsModel bsm;

void booz_sensors_model_init(void) {
  bsm.accel = v_get(AXIS_NB);
  bsm.accel->ve[AXIS_X] = 0.;
  bsm.accel->ve[AXIS_Y] = 0.;
  bsm.accel->ve[AXIS_Z] = 0.;

  bsm.accel_sensitivity = m_get(AXIS_NB, AXIS_NB);
  m_zero(bsm.accel_sensitivity);
  bsm.accel_sensitivity->me[AXIS_X][AXIS_X] = -(1024 * 32)  / (6. * 9.81);
  bsm.accel_sensitivity->me[AXIS_Y][AXIS_Y] =  (1024. * 32) / (6. * 9.81);
  bsm.accel_sensitivity->me[AXIS_Z][AXIS_Z] =  (1024. * 32) / (6. * 9.81);

  bsm.accel_neutral = v_get(AXIS_NB);
  bsm.accel_neutral->ve[AXIS_X] = 538 * 32;
  bsm.accel_neutral->ve[AXIS_Y] = 506 * 32;
  bsm.accel_neutral->ve[AXIS_Z] = 506 * 32;

  bsm.accel_noise_std_dev = v_get(AXIS_NB);
  bsm.accel_noise_std_dev->ve[AXIS_X] = 1e-1 * bsm.accel_sensitivity->me[AXIS_X][AXIS_X];
  bsm.accel_noise_std_dev->ve[AXIS_Y] = 1e-1 * bsm.accel_sensitivity->me[AXIS_Y][AXIS_Y];
  bsm.accel_noise_std_dev->ve[AXIS_Z] = 1e-1 * bsm.accel_sensitivity->me[AXIS_Z][AXIS_Z];

  bsm.accel_bias = v_get(AXIS_NB);
  bsm.accel_bias->ve[AXIS_P] = 1e-3 * bsm.accel_sensitivity->me[AXIS_X][AXIS_X];
  bsm.accel_bias->ve[AXIS_Q] = 1e-3 * bsm.accel_sensitivity->me[AXIS_Y][AXIS_Y];
  bsm.accel_bias->ve[AXIS_R] = 1e-3 * bsm.accel_sensitivity->me[AXIS_Z][AXIS_Z];

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

  bsm.gyro_noise_std_dev = v_get(AXIS_NB);
  bsm.gyro_noise_std_dev->ve[AXIS_P] = 5e-3 * bsm.gyro_sensitivity->me[AXIS_P][AXIS_P];
  bsm.gyro_noise_std_dev->ve[AXIS_Q] = 5e-3 * bsm.gyro_sensitivity->me[AXIS_Q][AXIS_Q];
  bsm.gyro_noise_std_dev->ve[AXIS_R] = 5e-3 * bsm.gyro_sensitivity->me[AXIS_R][AXIS_R];

  bsm.gyro_bias = v_get(AXIS_NB);
  bsm.gyro_bias->ve[AXIS_P] = 1e-5 * bsm.gyro_sensitivity->me[AXIS_P][AXIS_P];
  bsm.gyro_bias->ve[AXIS_Q] = 1e-5 * bsm.gyro_sensitivity->me[AXIS_Q][AXIS_Q];
  bsm.gyro_bias->ve[AXIS_R] = 1e-5 * bsm.gyro_sensitivity->me[AXIS_R][AXIS_R];
}


void booz_sensors_model_run(void) {

  /* compute forces */
  static VEC *accel_body = VNULL;
  accel_body = v_resize(accel_body, AXIS_NB);
#if 0
  accel_body =  booz_flight_model_get_forces_body_frame(accel_body);
  /* divide by mass */
  accel_body = sv_mlt(1./bfm.mass, accel_body, accel_body);
  /* get g in body frame */
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
  static VEC *g_body = VNULL;
  g_body = v_resize(g_body, AXIS_NB);
  g_body = mv_mlt(dcm, bfm.g_earth, g_body);
  accel_body = v_sub(g_body, accel_body, accel_body);
#else
  accel_body->ve[AXIS_X] = -9.81 * sin(bfm.state->ve[BFMS_THETA]);
  accel_body->ve[AXIS_Y] = 9.81 * sin(bfm.state->ve[BFMS_PHI]) * cos(bfm.state->ve[BFMS_THETA]);
  accel_body->ve[AXIS_Z] = 9.81 * cos(bfm.state->ve[BFMS_PHI]) * cos(bfm.state->ve[BFMS_THETA]);
#endif

  //  printf("sim accel %f %f %f\n",accel_body->ve[AXIS_X] ,accel_body->ve[AXIS_Y] ,accel_body->ve[AXIS_Z]); 
  /* compute accel reading */
  bsm.accel = mv_mlt(bsm.accel_sensitivity, accel_body, bsm.accel); 
  bsm.accel = v_add(bsm.accel, bsm.accel_neutral, bsm.accel);
  /* compute accel error readings */
  /* gaussian noise */
  static VEC *accel_error = VNULL;
  accel_error = v_resize(accel_error, AXIS_NB);
  accel_error = v_rand(accel_error);
  static VEC *one = VNULL;
  one = v_resize(one, AXIS_NB);
  one = v_ones(one);
  accel_error = v_mltadd(one, accel_error, -2., accel_error); 
  accel_error = v_star(accel_error, bsm.accel_noise_std_dev, accel_error);
  /* constant bias  */
  accel_error = v_add(accel_error, bsm.accel_bias, accel_error); 
  /* add per accel error reading */
  bsm.accel =  v_add(bsm.accel, accel_error, bsm.accel); 
  /* round signal to account for adc discretisation */
  bsm.accel->ve[AXIS_X] = round( bsm.accel->ve[AXIS_X]);
  bsm.accel->ve[AXIS_Y] = round( bsm.accel->ve[AXIS_Y]);
  bsm.accel->ve[AXIS_Z] = round( bsm.accel->ve[AXIS_Z]);

  //  printf("sim adc %f %f %f\n",bsm.accel->ve[AXIS_X] ,bsm.accel->ve[AXIS_Y] ,bsm.accel->ve[AXIS_Z]); 

  /* extract rotational speed from flight model state */
  static VEC *rate_body = VNULL;
  rate_body = v_resize(rate_body, AXIS_NB);
  rate_body->ve[AXIS_P] = bfm.state->ve[BFMS_P];
  rate_body->ve[AXIS_Q] = bfm.state->ve[BFMS_Q];
  rate_body->ve[AXIS_R] = bfm.state->ve[BFMS_R];

  /* compute gyros readings */
  bsm.gyro = mv_mlt(bsm.gyro_sensitivity, rate_body, bsm.gyro); 
  bsm.gyro = v_add(bsm.gyro, bsm.gyro_neutral, bsm.gyro); 

  /* compute gyro error readinf*/
  /* gaussian noise */
  static VEC *gyro_error = VNULL;
  gyro_error = v_resize(gyro_error, AXIS_NB);
  gyro_error = v_rand(gyro_error);
  gyro_error = v_mltadd(one, gyro_error, -2., gyro_error); 
  gyro_error = v_star(gyro_error, bsm.gyro_noise_std_dev, gyro_error);
  /* constant bias  */
  gyro_error = v_add(bsm.gyro_bias, gyro_error, gyro_error); 
  /* add per gyro error reading */
  bsm.gyro =  v_add(bsm.gyro, gyro_error, bsm.gyro); 
  /* round signal to account for adc discretisation */
  bsm.gyro->ve[AXIS_P] = round( bsm.gyro->ve[AXIS_P]);
  bsm.gyro->ve[AXIS_Q] = round( bsm.gyro->ve[AXIS_Q]);
  bsm.gyro->ve[AXIS_R] = round( bsm.gyro->ve[AXIS_R]);

}
