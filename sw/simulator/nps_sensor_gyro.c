#include "nps_sensor_gyro.h"

#include "airframe.h"
#include "nps_fdm.h"
#include NPS_SENSORS_PARAMS


void  nps_sensor_gyro_init(struct NpsSensorGyro* gyro, double time) {
  VECT3_ASSIGN(gyro->value, 0., 0., 0.);
  gyro->resolution = NPS_GYRO_RESOLUTION;
  FLOAT_MAT33_DIAG(gyro->sensitivity, 
		   NPS_GYRO_SENSITIVITY_PP, NPS_GYRO_SENSITIVITY_QQ, NPS_GYRO_SENSITIVITY_RR);
  VECT3_ASSIGN(gyro->neutral, 
	       NPS_GYRO_NEUTRAL_P, NPS_GYRO_NEUTRAL_Q, NPS_GYRO_NEUTRAL_R);
  VECT3_ASSIGN(gyro->noise_std_dev, 
	       NPS_GYRO_NOISE_STD_DEV_P, NPS_GYRO_NOISE_STD_DEV_Q, NPS_GYRO_NOISE_STD_DEV_R);
  VECT3_ASSIGN(gyro->bias_initial, 
	       NPS_GYRO_BIAS_INITIAL_P, NPS_GYRO_BIAS_INITIAL_Q, NPS_GYRO_BIAS_INITIAL_R);
  VECT3_ASSIGN(gyro->bias_random_walk_std_dev, 
	       NPS_GYRO_BIAS_RANDOM_WALK_STD_DEV_P, 
	       NPS_GYRO_BIAS_RANDOM_WALK_STD_DEV_Q, 
	       NPS_GYRO_BIAS_RANDOM_WALK_STD_DEV_R);
  gyro->next_update = time;
  gyro->data_available = FALSE;
}

void nps_sensor_gyro_run_step(struct NpsSensorGyro* gyro, double time, struct DoubleRMat* body_to_imu) {
  
  if (time < gyro->next_update)
    return;

  /* rotate rate to IMU frame */
  DoubleVect3 rate_body = {fdm.body_rate->ve[0], fdm.body_rate->ve[1], fdm.body_rate->ve[2]}; 
  DoubleVect3 rate_imu;
  MAT33_VECT3_MUL(rate_imu, *body_to_imu, rate_body);
  /* compute gyros readings */
  MAT33_VECT3_MUL(gyro->value, gyro->sensitivity, rate_imu);
  /* compute gyro error readings */

  /* round signal to account for adc discretisation */
  DOUBLE_VECT3_ROUND(gyro->value);
  /* saturate                                       */
  VECT3_BOUND_CUBE(gyro->value, 0, gyro->resolution); 
  
#if 0
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
#endif

  gyro->next_update += NPS_GYRO_DT;
  gyro->data_available = TRUE;
}



