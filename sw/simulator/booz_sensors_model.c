#include "booz_sensors_model.h"

#include <math.h>

#include "std.h"
#include "booz_flight_model.h"
#include "booz_flight_model_utils.h"

struct BoozSensorsModel bsm;

static void booz_sensors_model_accel_init(void);
static void booz_sensors_model_accel_run(MAT* dcm);

static void booz_sensors_model_gyro_init(void);
static void booz_sensors_model_gyro_run(double dt);

static void booz_sensors_model_mag_init(void);
static void booz_sensors_model_mag_run(MAT* dcm);

static void booz_sensors_model_gps_init(void);
static void booz_sensors_model_gps_run(double dt, MAT* dcm_t);

static VEC* v_add_gaussian_noise(VEC* in, VEC* std_dev, VEC* out);
static VEC* v_update_random_walk(VEC* in, VEC* std_dev, double dt, VEC* out);

void booz_sensors_model_init(void) {
  booz_sensors_model_accel_init();
  booz_sensors_model_gyro_init();
  booz_sensors_model_mag_init();
  booz_sensors_model_gps_init();
} 

void booz_sensors_model_run(double dt) {

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

  booz_sensors_model_accel_run(dcm);
  booz_sensors_model_gyro_run(dt);
  booz_sensors_model_mag_run(dcm);
  booz_sensors_model_gps_run(dt, dcm_t);
} 

static void booz_sensors_model_accel_init(void) {

  bsm.accel = v_get(AXIS_NB);
  bsm.accel->ve[AXIS_X] = 0.;
  bsm.accel->ve[AXIS_Y] = 0.;
  bsm.accel->ve[AXIS_Z] = 0.;
  bsm.accel_resolution = 1024 * 32;

  bsm.accel_sensitivity = m_get(AXIS_NB, AXIS_NB);
  m_zero(bsm.accel_sensitivity);
  bsm.accel_sensitivity->me[AXIS_X][AXIS_X] = -(double)(bsm.accel_resolution) / (6. * 9.81);
  bsm.accel_sensitivity->me[AXIS_Y][AXIS_Y] =  (double)(bsm.accel_resolution) / (6. * 9.81);
  bsm.accel_sensitivity->me[AXIS_Z][AXIS_Z] =  (double)(bsm.accel_resolution) / (6. * 9.81);

  bsm.accel_neutral = v_get(AXIS_NB);
  bsm.accel_neutral->ve[AXIS_X] = 538 * 32;
  bsm.accel_neutral->ve[AXIS_Y] = 506 * 32;
  bsm.accel_neutral->ve[AXIS_Z] = 506 * 32;

  bsm.accel_noise_std_dev = v_get(AXIS_NB);
  bsm.accel_noise_std_dev->ve[AXIS_X] = 2e-1; /* m2s-4 */
  bsm.accel_noise_std_dev->ve[AXIS_Y] = 2e-1; /* m2s-4 */
  bsm.accel_noise_std_dev->ve[AXIS_Z] = 2e-1; /* m2s-4 */

  bsm.accel_bias = v_get(AXIS_NB);
  bsm.accel_bias->ve[AXIS_P] = 1e-3; /* ms-2 */
  bsm.accel_bias->ve[AXIS_Q] = 1e-3; /* ms-2 */
  bsm.accel_bias->ve[AXIS_R] = 1e-3; /* ms-2 */

}


static void booz_sensors_model_gyro_init(void) {

  bsm.gyro = v_get(AXIS_NB);
  bsm.gyro->ve[AXIS_P] = 0.;
  bsm.gyro->ve[AXIS_Q] = 0.;
  bsm.gyro->ve[AXIS_R] = 0.;
  bsm.gyro_resolution = 65536;

  bsm.gyro_sensitivity = m_get(AXIS_NB, AXIS_NB);
  m_zero(bsm.gyro_sensitivity);
  bsm.gyro_sensitivity->me[AXIS_P][AXIS_P] = (double)bsm.gyro_resolution / (2.*RadOfDeg(-413.41848)); /* degres/s - nominal 300 */
  bsm.gyro_sensitivity->me[AXIS_Q][AXIS_Q] = (double)bsm.gyro_resolution / (2.*RadOfDeg(-403.65564)); /* degres/s - nominal 300 */
  bsm.gyro_sensitivity->me[AXIS_R][AXIS_R] = (double)bsm.gyro_resolution / (2.*RadOfDeg( 395.01929)); /* degres/s - nominal 300 */

  bsm.gyro_neutral = v_get(AXIS_NB);
  bsm.gyro_neutral->ve[AXIS_P] = (double)bsm.gyro_resolution * 0.6238556; /* ratio of full scale - nominal 0.5 */
  bsm.gyro_neutral->ve[AXIS_Q] = (double)bsm.gyro_resolution * 0.6242371; /* ratio of full scale - nominal 0.5 */
  bsm.gyro_neutral->ve[AXIS_R] = (double)bsm.gyro_resolution * 0.6035156; /* ratio of full scale - nominal 0.5 */

  bsm.gyro_noise_std_dev = v_get(AXIS_NB);
  bsm.gyro_noise_std_dev->ve[AXIS_P] = RadOfDeg(1.);
  bsm.gyro_noise_std_dev->ve[AXIS_Q] = RadOfDeg(1.);
  bsm.gyro_noise_std_dev->ve[AXIS_R] = RadOfDeg(1.);

  bsm.gyro_bias_initial = v_get(AXIS_NB);
  bsm.gyro_bias_initial->ve[AXIS_P] = RadOfDeg( 1.0);
  bsm.gyro_bias_initial->ve[AXIS_Q] = RadOfDeg(-0.25);
  bsm.gyro_bias_initial->ve[AXIS_R] = RadOfDeg( 0.5);

  bsm.gyro_bias_random_walk_std_dev = v_get(AXIS_NB);
  bsm.gyro_bias_random_walk_std_dev->ve[AXIS_P] =  RadOfDeg(5.e-1);
  bsm.gyro_bias_random_walk_std_dev->ve[AXIS_Q] =  RadOfDeg(5.e-1);
  bsm.gyro_bias_random_walk_std_dev->ve[AXIS_R] =  RadOfDeg(5.e-1);

  bsm.gyro_bias_random_walk_value = v_get(AXIS_NB);
  bsm.gyro_bias_random_walk_value->ve[AXIS_P] = bsm.gyro_bias_initial->ve[AXIS_P];
  bsm.gyro_bias_random_walk_value->ve[AXIS_Q] = bsm.gyro_bias_initial->ve[AXIS_Q];
  bsm.gyro_bias_random_walk_value->ve[AXIS_R] = bsm.gyro_bias_initial->ve[AXIS_R];

}


static void booz_sensors_model_mag_init(void) {

  bsm.mag = v_get(AXIS_NB);
  bsm.mag->ve[AXIS_X] = 0.;
  bsm.mag->ve[AXIS_Y] = 0.;
  bsm.mag->ve[AXIS_Z] = 0.;
  bsm.mag_resolution = 4096;

  bsm.mag_sensitivity = m_get(AXIS_NB, AXIS_NB);
  m_zero(bsm.mag_sensitivity);
  bsm.mag_sensitivity->me[AXIS_X][AXIS_X] = -(double)bsm.mag_resolution / 6.;
  bsm.mag_sensitivity->me[AXIS_Y][AXIS_Y] = -(double)bsm.mag_resolution / 6.;
  bsm.mag_sensitivity->me[AXIS_Z][AXIS_Z] = (double)bsm.mag_resolution / 6.;

  bsm.mag_neutral = v_get(AXIS_NB);
  bsm.mag_neutral->ve[AXIS_X] = 0.;
  bsm.mag_neutral->ve[AXIS_Y] = 0.;
  bsm.mag_neutral->ve[AXIS_Z] = 0.;

  bsm.mag_noise_std_dev = v_get(AXIS_NB);
  bsm.mag_noise_std_dev->ve[AXIS_X] = 2e-1;
  bsm.mag_noise_std_dev->ve[AXIS_Y] = 2e-1;
  bsm.mag_noise_std_dev->ve[AXIS_Z] = 2e-1;

}

static void booz_sensors_model_gps_init(void) {

  bsm.speed_sensor = v_get(AXIS_NB);
  v_zero(bsm.speed_sensor);

  bsm.speed_noise_std_dev = v_get(AXIS_NB);
  bsm.speed_noise_std_dev->ve[AXIS_X] = 1e-1;
  bsm.speed_noise_std_dev->ve[AXIS_Y] = 1e-1;
  bsm.speed_noise_std_dev->ve[AXIS_Z] = 1e-1;

  bsm.pos_sensor = v_get(AXIS_NB);
  v_zero(bsm.pos_sensor);

  bsm.pos_noise_std_dev = v_get(AXIS_NB);
  bsm.pos_noise_std_dev->ve[AXIS_X] = 1e-1;
  bsm.pos_noise_std_dev->ve[AXIS_Y] = 1e-1;
  bsm.pos_noise_std_dev->ve[AXIS_Z] = 1e-1;

  bsm.pos_bias_initial = v_get(AXIS_NB);
  bsm.pos_bias_initial->ve[AXIS_X] = 1e-1;
  bsm.pos_bias_initial->ve[AXIS_Y] = 1e-1;
  bsm.pos_bias_initial->ve[AXIS_Z] = 1e-1;

  bsm.pos_bias_random_walk_std_dev = v_get(AXIS_NB);
  bsm.pos_bias_random_walk_std_dev->ve[AXIS_X] = 1e-1;
  bsm.pos_bias_random_walk_std_dev->ve[AXIS_Y] = 1e-1;
  bsm.pos_bias_random_walk_std_dev->ve[AXIS_Z] = 1e-1;

  bsm.pos_bias_random_walk_value = v_get(AXIS_NB);
  bsm.pos_bias_random_walk_value->ve[AXIS_X] = bsm.pos_bias_initial->ve[AXIS_X];
  bsm.pos_bias_random_walk_value->ve[AXIS_Y] = bsm.pos_bias_initial->ve[AXIS_Y];
  bsm.pos_bias_random_walk_value->ve[AXIS_Z] = bsm.pos_bias_initial->ve[AXIS_Z];
}


#define RoundSensor(_sensor) {				\
    _sensor->ve[AXIS_X] = rint(_sensor->ve[AXIS_X]);	\
    _sensor->ve[AXIS_Y] = rint(_sensor->ve[AXIS_Y]);	\
    _sensor->ve[AXIS_Z] = rint(_sensor->ve[AXIS_Z]);	\
  }

#define BoundSensor(_sensor, _min, _max) {			    \
  if ( _sensor->ve[AXIS_X] < _min) _sensor->ve[AXIS_X] = _min; \
  if ( _sensor->ve[AXIS_X] > _max) _sensor->ve[AXIS_X] = _max; \
  if ( _sensor->ve[AXIS_Y] < _min) _sensor->ve[AXIS_Y] = _min; \
  if ( _sensor->ve[AXIS_Y] > _max) _sensor->ve[AXIS_Y] = _max; \
  if ( _sensor->ve[AXIS_Z] < _min) _sensor->ve[AXIS_Z] = _min; \
  if ( _sensor->ve[AXIS_Z] > _max) _sensor->ve[AXIS_Z] = _max; \
  }

static void booz_sensors_model_accel_run( MAT* dcm ) {

  /*  */
  static VEC* accel_body = VNULL;
  accel_body = v_resize(accel_body, AXIS_NB);
  accel_body = v_zero(accel_body);
#if 1
  /* get g in body frame */
  static VEC *g_body = VNULL;
  g_body = v_resize(g_body, AXIS_NB);
  g_body = mv_mlt(dcm, bfm.g_earth, g_body);

  /* add non inertial forces */
  /* extract body speed from state */
  static VEC *speed_body = VNULL;
  speed_body = v_resize(speed_body, AXIS_NB);
  speed_body->ve[AXIS_X] = bfm.state->ve[BFMS_U];
  speed_body->ve[AXIS_Y] = bfm.state->ve[BFMS_V];
  speed_body->ve[AXIS_Z] = bfm.state->ve[BFMS_W];
  /* extracts body rates from state */
  static VEC *rate_body = VNULL;
  rate_body = v_resize(rate_body, AXIS_NB);
  rate_body->ve[AXIS_P] = bfm.state->ve[BFMS_P];
  rate_body->ve[AXIS_Q] = bfm.state->ve[BFMS_Q];
  rate_body->ve[AXIS_R] = bfm.state->ve[BFMS_R];
  static VEC *fict_f = VNULL;
  fict_f = v_resize(fict_f, AXIS_NB);
  fict_f = out_prod(speed_body, rate_body, fict_f);
  //  fict_f = sv_mlt(bfm.mass, fict_f, fict_f);
  /* divide by mass */
  //  accel_body = sv_mlt(1./bfm.mass, accel_body, accel_body);
  accel_body = v_add(g_body, fict_f, accel_body);
#else
  printf(" accel_body # %f %f %f\n", accel_body->ve[AXIS_X], accel_body->ve[AXIS_Y], accel_body->ve[AXIS_Z]);
  accel_body->ve[AXIS_X] = -9.81 * sin(bfm.state->ve[BFMS_THETA]);
  accel_body->ve[AXIS_Y] = 9.81 * sin(bfm.state->ve[BFMS_PHI]) * cos(bfm.state->ve[BFMS_THETA]);
  accel_body->ve[AXIS_Z] = 9.81 * cos(bfm.state->ve[BFMS_PHI]) * cos(bfm.state->ve[BFMS_THETA]);
  printf(" accel_body ~ %f %f %f\n", accel_body->ve[AXIS_X], accel_body->ve[AXIS_Y], accel_body->ve[AXIS_Z]);
#endif

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
}


static void booz_sensors_model_gyro_run( double dt ) {
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
  			 bsm.gyro_bias_random_walk_std_dev, dt, 
  			 bsm.gyro_bias_random_walk_value);
  gyro_error = v_add(gyro_error, bsm.gyro_bias_random_walk_value, gyro_error); 
  /* scale to adc units FIXME : should use full adc gain ? sum ? */
  gyro_error->ve[AXIS_P] = gyro_error->ve[AXIS_P] * bsm.gyro_sensitivity->me[AXIS_P][AXIS_P];
  gyro_error->ve[AXIS_Q] = gyro_error->ve[AXIS_Q] * bsm.gyro_sensitivity->me[AXIS_Q][AXIS_Q];
  gyro_error->ve[AXIS_R] = gyro_error->ve[AXIS_R] * bsm.gyro_sensitivity->me[AXIS_R][AXIS_R];
  /* add per gyro error reading */
  bsm.gyro =  v_add(bsm.gyro, gyro_error, bsm.gyro); 
  /* round signal to account for adc discretisation */
  RoundSensor(bsm.gyro);
  /* saturation                                     */
  BoundSensor(bsm.gyro, 0, bsm.gyro_resolution); 
}


static void booz_sensors_model_mag_run( MAT* dcm ) {
  /* rotate h to body frame */
  static VEC *h_body = VNULL;
  h_body = v_resize(h_body, AXIS_NB);
  h_body = mv_mlt(dcm, bfm.h_earth, h_body);
  
  bsm.mag = mv_mlt(bsm.mag_sensitivity, h_body, bsm.mag); 
  bsm.mag = v_add(bsm.mag, bsm.mag_neutral, bsm.mag); 

  //  printf("h body %f %f %f\n", h_body->ve[AXIS_X], h_body->ve[AXIS_Y], h_body->ve[AXIS_Z]);
  //  printf("mag %f %f %f\n", bsm.mag->ve[AXIS_X], bsm.mag->ve[AXIS_Y], bsm.mag->ve[AXIS_Z]);
  /* round signal to account for adc discretisation */  
  RoundSensor(bsm.mag);
}


static void booz_sensors_model_gps_run( double dt, MAT* dcm_t ) {

  /* simulate speed sensor */
  /* extract body speed from state */
  static VEC *speed_body = VNULL;
  speed_body = v_resize(speed_body, AXIS_NB);
  speed_body->ve[AXIS_U] = bfm.state->ve[BFMS_U];
  speed_body->ve[AXIS_V] = bfm.state->ve[BFMS_V];
  speed_body->ve[AXIS_W] = bfm.state->ve[BFMS_W];
  /* convert to earth frame */
  bsm.speed_sensor = mv_mlt(dcm_t, speed_body, bsm.speed_sensor);
  /* add a gaussian noise */
  bsm.speed_sensor = v_add_gaussian_noise(bsm.speed_sensor, bsm.speed_noise_std_dev, bsm.speed_sensor);
  

  /* simulate position sensor */
  bsm.pos_sensor->ve[AXIS_X] = bfm.state->ve[BFMS_X];
  bsm.pos_sensor->ve[AXIS_Y] = bfm.state->ve[BFMS_Y];
  bsm.pos_sensor->ve[AXIS_Z] = bfm.state->ve[BFMS_Z];

  /* compute gyro error reading */
  static VEC *pos_error = VNULL;
  pos_error = v_resize(pos_error, AXIS_NB);
  pos_error = v_zero(pos_error);
  /* add a gaussian noise */
  pos_error = v_add_gaussian_noise(pos_error, bsm.pos_noise_std_dev, pos_error);
  /* update random walk bias */
  bsm.pos_bias_random_walk_value = 
    v_update_random_walk(bsm.pos_bias_random_walk_value, 
  			 bsm.pos_bias_random_walk_std_dev, dt, 
  			 bsm.pos_bias_random_walk_value);
  pos_error = v_add(pos_error, bsm.pos_bias_random_walk_value, pos_error); 
  /* add error reading */
  bsm.pos_sensor =  v_add(bsm.pos_sensor, pos_error, bsm.pos_sensor); 
  
}


static VEC* v_update_random_walk(VEC* in, VEC* std_dev, double dt, VEC* out) {
  static VEC *tmp = VNULL;
  tmp = v_resize(tmp, AXIS_NB);
  tmp = sv_mlt(dt, std_dev, tmp);
  out =  v_add_gaussian_noise(in, tmp, out);
  return out;
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
