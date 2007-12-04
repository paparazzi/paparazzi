#include "booz_sensors_model.h"
#include "booz_sensors_model_params.h"

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

static void booz_sensors_model_range_meter_init(void);
static void booz_sensors_model_range_meter_run();

static void booz_sensors_model_gps_init(void);
static void booz_sensors_model_gps_run(double dt, MAT* dcm_t);

static VEC* v_add_gaussian_noise(VEC* in, VEC* std_dev, VEC* out);
static VEC* v_update_random_walk(VEC* in, VEC* std_dev, double dt, VEC* out);
static void UpdateSensorLatency(VEC* cur_reading, GSList* history, 
				double latency, VEC* sensor_reading);

void booz_sensors_model_init(void) {
  booz_sensors_model_accel_init();
  booz_sensors_model_gyro_init();
  booz_sensors_model_mag_init();
  booz_sensors_model_range_meter_init();
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
  booz_sensors_model_range_meter_run();
  booz_sensors_model_gps_run(dt, dcm_t);
} 

static void booz_sensors_model_accel_init(void) {

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

}


static void booz_sensors_model_gyro_init(void) {

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
  bsm.mag_noise_std_dev->ve[AXIS_X] = 2e-2;
  bsm.mag_noise_std_dev->ve[AXIS_Y] = 2e-2;
  bsm.mag_noise_std_dev->ve[AXIS_Z] = 2e-2;

}

static void booz_sensors_model_range_meter_init(void) {
  bsm.range_meter = 0.;
  bsm.range_meter_resolution = BSM_RANGE_METER_RESOLUTION;
  bsm.range_meter_sensivity = BSM_RANGE_METER_SENSITIVITY;

}

static void booz_sensors_model_gps_init(void) {

  bsm.speed_sensor = v_get(AXIS_NB);
  v_zero(bsm.speed_sensor);

  bsm.speed_noise_std_dev = v_get(AXIS_NB);
  bsm.speed_noise_std_dev->ve[AXIS_X] = 1e-1;
  bsm.speed_noise_std_dev->ve[AXIS_Y] = 1e-1;
  bsm.speed_noise_std_dev->ve[AXIS_Z] = 1e-1;

  bsm.speed_latency = .25;
  bsm.speed_history = NULL;

  bsm.pos_sensor = v_get(AXIS_NB);
  v_zero(bsm.pos_sensor);

  bsm.pos_noise_std_dev = v_get(AXIS_NB);
  bsm.pos_noise_std_dev->ve[AXIS_X] = 3e-1;
  bsm.pos_noise_std_dev->ve[AXIS_Y] = 3e-1;
  bsm.pos_noise_std_dev->ve[AXIS_Z] = 3e-1;

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

  bsm.pos_latency = .25;
  bsm.pos_history = NULL;

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

#define CopyVect(_dest, _src) {			\
    _dest->ve[AXIS_X] = _src->ve[AXIS_X];	\
    _dest->ve[AXIS_Y] = _src->ve[AXIS_Y];	\
    _dest->ve[AXIS_Z] = _src->ve[AXIS_Z];	\
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

  /* compute mag error readings */  
  static VEC *mag_error = VNULL;
  mag_error = v_resize(mag_error, AXIS_NB);
  mag_error = v_zero(mag_error);
  /* add a gaussian noise */
  mag_error = v_add_gaussian_noise(mag_error, bsm.mag_noise_std_dev, mag_error);
  
  mag_error->ve[AXIS_X] = mag_error->ve[AXIS_X] * bsm.mag_sensitivity->me[AXIS_X][AXIS_X];
  mag_error->ve[AXIS_Y] = mag_error->ve[AXIS_Y] * bsm.mag_sensitivity->me[AXIS_Y][AXIS_Y];
  mag_error->ve[AXIS_Z] = mag_error->ve[AXIS_Z] * bsm.mag_sensitivity->me[AXIS_Z][AXIS_Z];

  /* add error */
  bsm.mag =  v_add(bsm.mag, mag_error, bsm.mag); 

  //  printf("h body %f %f %f\n", h_body->ve[AXIS_X], h_body->ve[AXIS_Y], h_body->ve[AXIS_Z]);
  //  printf("mag %f %f %f\n", bsm.mag->ve[AXIS_X], bsm.mag->ve[AXIS_Y], bsm.mag->ve[AXIS_Z]);
  /* round signal to account for adc discretisation */  
  RoundSensor(bsm.mag);
}


static void booz_sensors_model_range_meter_run() {
  double dz = bfm.state->ve[BFMS_Z];
  if (dz > 0.) dz = 0.;
  double dx = dz * tan(bfm.state->ve[BFMS_THETA]);
  double dy = dz * tan(bfm.state->ve[BFMS_PHI]);
  double dist = sqrt( dx*dx + dy*dy + dz*dz);
  dist *= bsm.range_meter_sensivity;
  /* add gaussian noise */

  if (dist > BSM_RANGE_METER_MAX_RANGE)
    dist = BSM_RANGE_METER_MAX_RANGE;
  dist = rint(dist);
  bsm.range_meter = dist;
}


static void booz_sensors_model_gps_run( double dt, MAT* dcm_t ) {

  /* simulate speed sensor */
  /* extract body speed from state */
  static VEC *speed_body = VNULL;
  speed_body = v_resize(speed_body, AXIS_NB);
  BoozFlighModelGetSpeed(speed_body);
  static VEC *cur_speed_reading = VNULL;
  cur_speed_reading = v_resize(cur_speed_reading, AXIS_NB);
  /* convert to earth frame */
  cur_speed_reading = mv_mlt(dcm_t, speed_body, cur_speed_reading);
  /* add a gaussian noise */
  cur_speed_reading = v_add_gaussian_noise(cur_speed_reading, bsm.speed_noise_std_dev, 
					   cur_speed_reading);
  UpdateSensorLatency(cur_speed_reading, bsm.speed_history, bsm.speed_latency, bsm.speed_sensor);

  /* simulate position sensor */
  static VEC *cur_pos_reading = VNULL;
  cur_pos_reading = v_resize(cur_pos_reading, AXIS_NB);
  /* extract pos from state */
  BoozFlighModelGetPos(cur_pos_reading);

  /* compute position error reading */
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
  cur_pos_reading = v_add(cur_pos_reading, pos_error, cur_pos_reading); 

  UpdateSensorLatency(cur_pos_reading, bsm.pos_history, bsm.pos_latency, bsm.pos_sensor);

}


static void UpdateSensorLatency(VEC* cur_reading, GSList* history, 
				double latency, VEC* sensor_reading) {
  /* add new reading */
  struct BoozDatedSensor* cur_read = g_new(struct BoozDatedSensor, 1);
  cur_read->time = bfm.time;
  cur_read->value = v_get(AXIS_NB);
  CopyVect(cur_read->value, cur_reading);
  history = g_slist_prepend(history, cur_read);
  /* remove old readings */
  GSList* last =  g_slist_last(history);
  while (last && 
	 ((struct BoozDatedSensor*)last->data)->time < bfm.time - latency) {
    history = g_slist_remove_link(history, last);
    v_free(((struct BoozDatedSensor*)last->data)->value);
    g_free((struct BoozDatedSensor*)last->data);
    g_slist_free(last);
    last =  g_slist_last(history);
  }
  /* update sensor        */
  CopyVect(sensor_reading, ((struct BoozDatedSensor*)last->data)->value);
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
  out = v_add(in, tmp, out);
  return out;
}
