#include "nps_sensor_mag.h"

#include "generated/airframe.h"
#include "nps_fdm.h"
#include "nps_sensors.h"
#include "math/pprz_algebra_int.h"

void nps_sensor_mag_init(struct NpsSensorMag *mag, double time)
{
  VECT3_ASSIGN(mag->value, 0., 0., 0.);
  mag->min = NPS_MAG_MIN;
  mag->max = NPS_MAG_MAX;
  FLOAT_MAT33_DIAG(mag->sensitivity,
                   NPS_MAG_SENSITIVITY_XX, NPS_MAG_SENSITIVITY_YY, NPS_MAG_SENSITIVITY_ZZ);
  VECT3_ASSIGN(mag->neutral,
               NPS_MAG_NEUTRAL_X, NPS_MAG_NEUTRAL_Y, NPS_MAG_NEUTRAL_Z);
  VECT3_ASSIGN(mag->noise_std_dev,
               NPS_MAG_NOISE_STD_DEV_X, NPS_MAG_NOISE_STD_DEV_Y, NPS_MAG_NOISE_STD_DEV_Z);
  struct DoubleEulers imu_to_sensor_eulers =
    { NPS_MAG_IMU_TO_SENSOR_PHI, NPS_MAG_IMU_TO_SENSOR_THETA, NPS_MAG_IMU_TO_SENSOR_PSI };
  double_rmat_of_eulers(&(mag->imu_to_sensor_rmat), &imu_to_sensor_eulers);
  mag->next_update = time;
  mag->data_available = FALSE;
}

void nps_sensor_mag_run_step(struct NpsSensorMag *mag, double time, struct DoubleRMat *body_to_imu)
{

  if (time < mag->next_update) {
    return;
  }

  /* transform magnetic field to body frame */
  struct DoubleVect3 h_body;
  double_quat_vmult(&h_body, &fdm.ltp_to_body_quat, &fdm.ltp_h);

  /* transform to imu frame */
  struct DoubleVect3 h_imu;
  MAT33_VECT3_MUL(h_imu, *body_to_imu, h_body);

  /* transform to sensor frame */
  struct DoubleVect3 h_sensor;
  MAT33_VECT3_MUL(h_sensor, mag->imu_to_sensor_rmat, h_imu);

  /* compute magnetometer reading */
  MAT33_VECT3_MUL(mag->value, mag->sensitivity, h_sensor);
  VECT3_ADD(mag->value, mag->neutral);
  /* FIXME: ADD error reading */

  /* round signal to account for adc discretisation */
  DOUBLE_VECT3_ROUND(mag->value);
  /* saturate                                       */
  VECT3_BOUND_CUBE(mag->value, mag->min, mag->max);

  mag->next_update += NPS_MAG_DT;
  mag->data_available = TRUE;
}

