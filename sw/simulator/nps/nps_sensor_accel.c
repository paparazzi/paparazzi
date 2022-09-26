#include "nps_sensor_accel.h"

#include "nps_fdm.h"
#include "nps_random.h"
#include "nps_sensors.h"
#include "math/pprz_algebra_int.h"

void nps_sensor_accel_init(struct NpsSensorAccel *accel, double time)
{
  FLOAT_VECT3_ZERO(accel->value);
  accel->min = NPS_ACCEL_MIN;
  accel->max = NPS_ACCEL_MAX;
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

void nps_sensor_accel_run_step(struct NpsSensorAccel *accel, double time, struct DoubleRMat *body_to_imu)
{
  if (time < accel->next_update) {
    return;
  }

  /* transform to imu frame */
  struct DoubleVect3 accelero_imu;
  MAT33_VECT3_MUL(accelero_imu, *body_to_imu, fdm.body_accel);

  /* compute accelero readings */
  MAT33_VECT3_MUL(accel->value, accel->sensitivity, accelero_imu);
  VECT3_ADD(accel->value, accel->neutral);

  /* Compute sensor error */
  struct DoubleVect3 accelero_error;
  /* constant bias */
  VECT3_COPY(accelero_error, accel->bias);
  /* white noise   */
  double_vect3_add_gaussian_noise(&accelero_error, &accel->noise_std_dev);
  /* scale */
  struct DoubleVect3 gain = {accel->sensitivity.m[0], accel->sensitivity.m[4], accel->sensitivity.m[8]};
  VECT3_EW_MUL(accelero_error, accelero_error, gain);
  /* add error */
  VECT3_ADD(accel->value, accelero_error);

  /* round signal to account for adc discretisation */
  DOUBLE_VECT3_ROUND(accel->value);
  /* saturate                                       */
  VECT3_BOUND_CUBE(accel->value, accel->min, accel->max);

  accel->next_update += NPS_ACCEL_DT;
  accel->data_available = TRUE;
}

