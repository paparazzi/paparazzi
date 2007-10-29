#include "booz_sensors_model.h"

#include <math.h>

#include "booz_flight_model.h"
#include "booz_flight_model_utils.h"

struct BoozSensorsModel bsm;

static void booz_sensors_model_accel_init(void);
static void booz_sensors_model_accel_run(void);

static void booz_sensors_model_gyro_init(void);
static void booz_sensors_model_gyro_run(void);

static void booz_sensors_model_gps_init(void);
static void booz_sensors_model_gps_run(void);

static VEC* v_add_gaussian_noise(VEC* in, VEC* std_dev, VEC* out);

void booz_sensors_model_init(void) {
  booz_sensors_model_accel_init();
  booz_sensors_model_gyro_init();
  booz_sensors_model_gps_init();
} 

void booz_sensors_model_run(void) {
  booz_sensors_model_accel_run();
  booz_sensors_model_gyro_run();
  booz_sensors_model_gps_run();
} 

static void booz_sensors_model_accel_init(void) {

  bsm.accel = v_get(AXIS_NB);
  bsm.accel->ve[AXIS_X] = 0.;
  bsm.accel->ve[AXIS_Y] = 0.;
  bsm.accel->ve[AXIS_Z] = 0.;

  bsm.accel_sensitivity = m_get(AXIS_NB, AXIS_NB);
  m_zero(bsm.accel_sensitivity);
  bsm.accel_sensitivity->me[AXIS_X][AXIS_X] = -(1024. * 32) / (6. * 9.81);
  bsm.accel_sensitivity->me[AXIS_Y][AXIS_Y] =  (1024. * 32) / (6. * 9.81);
  bsm.accel_sensitivity->me[AXIS_Z][AXIS_Z] =  (1024. * 32) / (6. * 9.81);

  bsm.accel_neutral = v_get(AXIS_NB);
  bsm.accel_neutral->ve[AXIS_X] = 538 * 32;
  bsm.accel_neutral->ve[AXIS_Y] = 506 * 32;
  bsm.accel_neutral->ve[AXIS_Z] = 506 * 32;

  bsm.accel_noise_std_dev = v_get(AXIS_NB);
  bsm.accel_noise_std_dev->ve[AXIS_X] = 2e-1 * bsm.accel_sensitivity->me[AXIS_X][AXIS_X];
  bsm.accel_noise_std_dev->ve[AXIS_Y] = 2e-1 * bsm.accel_sensitivity->me[AXIS_Y][AXIS_Y];
  bsm.accel_noise_std_dev->ve[AXIS_Z] = 2e-1 * bsm.accel_sensitivity->me[AXIS_Z][AXIS_Z];

  bsm.accel_bias = v_get(AXIS_NB);
  bsm.accel_bias->ve[AXIS_P] = 1e-3 * bsm.accel_sensitivity->me[AXIS_X][AXIS_X];
  bsm.accel_bias->ve[AXIS_Q] = 1e-3 * bsm.accel_sensitivity->me[AXIS_Y][AXIS_Y];
  bsm.accel_bias->ve[AXIS_R] = 1e-3 * bsm.accel_sensitivity->me[AXIS_Z][AXIS_Z];

}


static void booz_sensors_model_gyro_init(void) {
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
  bsm.gyro_noise_std_dev->ve[AXIS_P] = 5e-2 * bsm.gyro_sensitivity->me[AXIS_P][AXIS_P];
  bsm.gyro_noise_std_dev->ve[AXIS_Q] = 5e-2 * bsm.gyro_sensitivity->me[AXIS_Q][AXIS_Q];
  bsm.gyro_noise_std_dev->ve[AXIS_R] = 5e-2 * bsm.gyro_sensitivity->me[AXIS_R][AXIS_R];

  bsm.gyro_bias = v_get(AXIS_NB);
  bsm.gyro_bias->ve[AXIS_P] =  2e-2 * bsm.gyro_sensitivity->me[AXIS_P][AXIS_P];
  bsm.gyro_bias->ve[AXIS_Q] = -2e-2 * bsm.gyro_sensitivity->me[AXIS_Q][AXIS_Q];
  bsm.gyro_bias->ve[AXIS_R] = -1e-2 * bsm.gyro_sensitivity->me[AXIS_R][AXIS_R];
}

static void booz_sensors_model_gps_init(void) {
  bsm.speed_sensor = v_get(AXIS_NB);
  v_zero(bsm.speed_sensor);
  bsm.pos_sensor = v_get(AXIS_NB);
  v_zero(bsm.pos_sensor);
}


static void booz_sensors_model_accel_run(void) {

  /* compute forces */
  static VEC *accel_body = VNULL;
  accel_body = v_resize(accel_body, AXIS_NB);
  accel_body =  booz_flight_model_get_forces_body_frame(accel_body);
  /* divide by mass */
  accel_body = sv_mlt(1./bfm.mass, accel_body, accel_body);
  //  printf(" accel_body %f %f %f\n", accel_body->ve[AXIS_X], accel_body->ve[AXIS_Y], accel_body->ve[AXIS_Z]);

#if 0
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
  static VEC *accel_error = VNULL;
  accel_error = v_resize(accel_error, AXIS_NB);
  accel_error = v_zero(accel_error);
  /* add a gaussian noise */
  accel_error = v_add_gaussian_noise(accel_error, bsm.accel_noise_std_dev, accel_error);
  /* constant bias  */
  accel_error = v_add(accel_error, bsm.accel_bias, accel_error); 
  /* add per accel error reading */
  bsm.accel =  v_add(bsm.accel, accel_error, bsm.accel); 
  /* round signal to account for adc discretisation */
  bsm.accel->ve[AXIS_X] = round( bsm.accel->ve[AXIS_X]);
  bsm.accel->ve[AXIS_Y] = round( bsm.accel->ve[AXIS_Y]);
  bsm.accel->ve[AXIS_Z] = round( bsm.accel->ve[AXIS_Z]);

  //  printf("sim adc %f %f %f\n",bsm.accel->ve[AXIS_X] ,bsm.accel->ve[AXIS_Y] ,bsm.accel->ve[AXIS_Z]); 
}


static void booz_sensors_model_gyro_run(void) {
  /* extract rotational speed from flight model state */
  static VEC *rate_body = VNULL;
  rate_body = v_resize(rate_body, AXIS_NB);
  rate_body->ve[AXIS_P] = bfm.state->ve[BFMS_P];
  rate_body->ve[AXIS_Q] = bfm.state->ve[BFMS_Q];
  rate_body->ve[AXIS_R] = bfm.state->ve[BFMS_R];

  /* compute gyros readings */
  bsm.gyro = mv_mlt(bsm.gyro_sensitivity, rate_body, bsm.gyro); 
  bsm.gyro = v_add(bsm.gyro, bsm.gyro_neutral, bsm.gyro); 

  /* compute gyro error reading */
  static VEC *gyro_error = VNULL;
  gyro_error = v_resize(gyro_error, AXIS_NB);
  gyro_error = v_zero(gyro_error);
  /* add a gaussian noise */
  gyro_error = v_add_gaussian_noise(gyro_error, bsm.gyro_noise_std_dev, gyro_error);
  /* add a constant bias  */
  gyro_error = v_add(bsm.gyro_bias, gyro_error, gyro_error); 
  /* add a random walk    */

  /* add per gyro error reading */
  bsm.gyro =  v_add(bsm.gyro, gyro_error, bsm.gyro); 
  /* round signal to account for adc discretisation */
  bsm.gyro->ve[AXIS_P] = round( bsm.gyro->ve[AXIS_P]);
  bsm.gyro->ve[AXIS_Q] = round( bsm.gyro->ve[AXIS_Q]);
  bsm.gyro->ve[AXIS_R] = round( bsm.gyro->ve[AXIS_R]);

}

static void booz_sensors_model_gps_run(void) {
  /* very wrong change me */
  bsm.speed_sensor->ve[AXIS_X] = bfm.state->ve[BFMS_U];
  bsm.speed_sensor->ve[AXIS_Y] = bfm.state->ve[BFMS_V];
  bsm.speed_sensor->ve[AXIS_Z] = bfm.state->ve[BFMS_W];
  
  bsm.pos_sensor->ve[AXIS_X] = bfm.state->ve[BFMS_X];
  bsm.pos_sensor->ve[AXIS_Y] = bfm.state->ve[BFMS_Y];
  bsm.pos_sensor->ve[AXIS_Z] = bfm.state->ve[BFMS_Z];

}


static VEC* v_add_gaussian_noise(VEC* in, VEC* std_dev, VEC* out) {
  static VEC *tmp = VNULL;
  tmp = v_resize(tmp, AXIS_NB);
  tmp = v_rand(tmp);
  static VEC *one = VNULL;
  one = v_resize(one, AXIS_NB);
  one = v_ones(one);
  tmp = v_mltadd(one, tmp, -2., tmp); 
  tmp = v_star(tmp, std_dev, tmp);
  out = v_add(out, tmp, out);
  return out;
}
