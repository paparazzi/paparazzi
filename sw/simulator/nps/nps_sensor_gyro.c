#include "nps_sensor_gyro.h"

#include "generated/airframe.h"
#include "nps_fdm.h"
#include NPS_SENSORS_PARAMS
#include "math/pprz_algebra_int.h"
#include "nps_random.h"

void  nps_sensor_gyro_init(struct NpsSensorGyro* gyro, double time) {
  FLOAT_VECT3_ZERO(gyro->value);
  gyro->min = NPS_GYRO_MIN;
  gyro->max = NPS_GYRO_MAX;
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
  FLOAT_VECT3_ZERO(gyro->bias_random_walk_value);
  gyro->next_update = time;
  gyro->data_available = FALSE;
}

void nps_sensor_gyro_run_step(struct NpsSensorGyro* gyro, double time, struct DoubleRMat* body_to_imu) {

  if (time < gyro->next_update)
    return;

  /* transform body rates to IMU frame */
  struct DoubleVect3* rate_body = (struct DoubleVect3*)(&fdm.body_inertial_rotvel);
  struct DoubleVect3 rate_imu;
  MAT33_VECT3_MUL(rate_imu, *body_to_imu, *rate_body );
  /* compute gyros readings */
  MAT33_VECT3_MUL(gyro->value, gyro->sensitivity, rate_imu);
  VECT3_ADD(gyro->value, gyro->neutral);
  /* compute gyro error readings */
  struct DoubleVect3 gyro_error;
  VECT3_COPY(gyro_error, gyro->bias_initial);
  double_vect3_add_gaussian_noise(&gyro_error, &gyro->noise_std_dev);
  double_vect3_update_random_walk(&gyro->bias_random_walk_value, &gyro->bias_random_walk_std_dev,
				  NPS_GYRO_DT, 5.);
  VECT3_ADD(gyro_error, gyro->bias_random_walk_value);

  struct DoubleVect3 gain = {NPS_GYRO_SENSITIVITY_PP, NPS_GYRO_SENSITIVITY_QQ, NPS_GYRO_SENSITIVITY_RR};
  VECT3_EW_MUL(gyro_error, gyro_error, gain);

  VECT3_ADD(gyro->value, gyro_error);

  /* round signal to account for adc discretisation */
  DOUBLE_VECT3_ROUND(gyro->value);
  /* saturate                                       */
  VECT3_BOUND_CUBE(gyro->value, gyro->min, gyro->max);

  gyro->next_update += NPS_GYRO_DT;
  gyro->data_available = TRUE;
}



