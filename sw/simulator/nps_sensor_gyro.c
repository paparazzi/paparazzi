//#include "nps_sensor_gyro.h"
#include "nps_sensors.h"

#include "airframe.h"
#include NPS_SENSORS_PARAMS


bool_t nps_sensor_gyro_available() {
  if (sensors.gyro.data_available) {
    sensors.gyro.data_available = FALSE;
    return TRUE;
  }
  return FALSE;
}

void  nps_sensor_gyro_init(double time) {
  VECT3_ASSIGN(sensors.gyro.value, 0., 0., 0.);
  sensors.gyro.resolution = NPS_GYRO_RESOLUTION;
  FLOAT_MAT33_ZERO(sensors.gyro.sensitivity);
  MAT33_ELMT(sensors.gyro.sensitivity, 0, 0) = NPS_GYRO_SENSITIVITY_PP;
  MAT33_ELMT(sensors.gyro.sensitivity, 1, 1) = NPS_GYRO_SENSITIVITY_QQ;
  MAT33_ELMT(sensors.gyro.sensitivity, 2, 2) = NPS_GYRO_SENSITIVITY_RR;
  VECT3_ASSIGN(sensors.gyro.neutral, 
	       NPS_GYRO_NEUTRAL_P, NPS_GYRO_NEUTRAL_Q, NPS_GYRO_NEUTRAL_R);
  VECT3_ASSIGN(sensors.gyro.noise_std_dev, 
	       NPS_GYRO_NOISE_STD_DEV_P, NPS_GYRO_NOISE_STD_DEV_Q, NPS_GYRO_NOISE_STD_DEV_R);
  VECT3_ASSIGN(sensors.gyro.bias_initial, 
	       NPS_GYRO_BIAS_INITIAL_P, NPS_GYRO_BIAS_INITIAL_Q, NPS_GYRO_BIAS_INITIAL_R);
  VECT3_ASSIGN(sensors.gyro.bias_random_walk_std_dev, 
	       NPS_GYRO_BIAS_RANDOM_WALK_STD_DEV_P, 
	       NPS_GYRO_BIAS_RANDOM_WALK_STD_DEV_Q, 
	       NPS_GYRO_BIAS_RANDOM_WALK_STD_DEV_R);
  sensors.gyro.next_update = time;
  sensors.gyro.data_available = FALSE;
}

void booz_sensors_model_gyro_run( double time ) {
  if (time < sensors.gyro.next_update)
    return;

#if 0
  /* rotate to IMU frame */
  /* convert to imu frame */
  static VEC* rate_imu = VNULL;
  rate_imu = v_resize(rate_imu, AXIS_NB);
  mv_mlt(bsm.body_to_imu, bfm.ang_rate_body, rate_imu);
  /* compute gyros readings */
  bsm.gyro = mv_mlt(bsm.gyro_sensitivity, rate_imu, bsm.gyro); 
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
#endif

  sensors.gyro.next_update += NPS_GYRO_DT;
  sensors.gyro.data_available = TRUE;
}



