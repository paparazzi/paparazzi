#include "nps_sensor_accel.h"

#include "airframe.h"
#include "nps_fdm.h"
#include NPS_SENSORS_PARAMS

void   nps_sensor_accel_init(struct NpsSensorAccel* accel, double time) {
  VECT3_ASSIGN(accel->value, 0., 0., 0.);
  accel->resolution = NPS_ACCEL_RESOLUTION;
  FLOAT_MAT33_DIAG(accel->sensitivity, 
		   NPS_ACCEL_SENSITIVITY_XX, NPS_ACCEL_SENSITIVITY_YY, NPS_ACCEL_SENSITIVITY_ZZ);
  VECT3_ASSIGN(accel->neutral, 
	       NPS_ACCEL_NEUTRAL_X, NPS_ACCEL_NEUTRAL_Y, NPS_ACCEL_NEUTRAL_Z);
  VECT3_ASSIGN(accel->noise_std_dev, 
	       NPS_ACCEL_NOISE_STD_DEV_X, NPS_ACCEL_NOISE_STD_DEV_Y, NPS_ACCEL_NOISE_STD_DEV_Z);
  VECT3_ASSIGN(accel->bias, 
	       NPS_ACCEL_BIAS_X, NPS_ACCEL_BIAS_Y, NPS_ACCEL_BIAS_Z);
  accel->next_update = time;
  accel->data_available = FALSE;
}

void   nps_sensor_accel_run_step(struct NpsSensorAccel* accel, double time, struct DoubleRMat* body_to_imu) {

  if (time < accel->next_update)
    return;
  
  /* transform gravity to body frame */
  struct DoubleVect3 g_body;
  FLOAT_QUAT_VMULT(g_body, fdm.ltp_to_body_quat, fdm.g_ltp);
  
  /* substract gravity to acceleration in body frame */
  struct DoubleVect3 accelero_body;
  FLOAT_VECT3_DIFF(accelero_body, fdm.body_accel, g_body);
  
  /* transform to imu frame */
  struct DoubleVect3 accelero_imu;
  MAT33_VECT3_MUL(accelero_imu, *body_to_imu, accelero_body );
  
  /* compute accelero readings */
  MAT33_VECT3_MUL(accel->value, accel->sensitivity, accelero_imu);
  
  /* FIXME: ADD error reading */
  
  /* round signal to account for adc discretisation */
  DOUBLE_VECT3_ROUND(accel->value);
  /* saturate                                       */
  VECT3_BOUND_CUBE(accel->value, 0, accel->resolution); 
  
  accel->next_update += NPS_ACCEL_DT;
  accel->data_available = TRUE;
}

