#include "nps_sensors.h"

#include "generated/airframe.h"
#include NPS_SENSORS_PARAMS

struct NpsSensors sensors;

void nps_sensors_init(double time)
{

  struct DoubleEulers body_to_imu_eulers =
  { NPS_BODY_TO_IMU_PHI, NPS_BODY_TO_IMU_THETA, NPS_BODY_TO_IMU_PSI };
  double_rmat_of_eulers(&sensors.body_to_imu_rmat, &body_to_imu_eulers);

  nps_sensor_gyro_init(&sensors.gyro, time);
  nps_sensor_accel_init(&sensors.accel, time);
  nps_sensor_mag_init(&sensors.mag, time);
  nps_sensor_baro_init(&sensors.baro, time);
  nps_sensor_gps_init(&sensors.gps, time);
  nps_sensor_sonar_init(&sensors.sonar, time);
  nps_sensor_airspeed_init(&sensors.airspeed, time);
  nps_sensor_temperature_init(&sensors.temp, time);
  nps_sensor_aoa_init(&sensors.aoa, time);
  nps_sensor_sideslip_init(&sensors.sideslip,time);
}


void nps_sensors_run_step(double time)
{
  nps_sensor_gyro_run_step(&sensors.gyro, time, &sensors.body_to_imu_rmat);
  nps_sensor_accel_run_step(&sensors.accel, time, &sensors.body_to_imu_rmat);
  nps_sensor_mag_run_step(&sensors.mag, time, &sensors.body_to_imu_rmat);
  nps_sensor_baro_run_step(&sensors.baro, time);
  nps_sensor_gps_run_step(&sensors.gps, time);
  nps_sensor_sonar_run_step(&sensors.sonar, time);
  nps_sensor_airspeed_run_step(&sensors.airspeed, time);
  nps_sensor_temperature_run_step(&sensors.temp, time);
  nps_sensor_aoa_run_step(&sensors.aoa, time);
  nps_sensor_sideslip_run_step(&sensors.sideslip,time);
}


bool nps_sensors_gyro_available(void)
{
  if (sensors.gyro.data_available) {
    sensors.gyro.data_available = FALSE;
    return TRUE;
  }
  return FALSE;
}

bool nps_sensors_mag_available(void)
{
  if (sensors.mag.data_available) {
    sensors.mag.data_available = FALSE;
    return TRUE;
  }
  return FALSE;
}

bool nps_sensors_baro_available(void)
{
  if (sensors.baro.data_available) {
    sensors.baro.data_available = FALSE;
    return TRUE;
  }
  return FALSE;
}

bool nps_sensors_gps_available(void)
{
  if (sensors.gps.data_available) {
    sensors.gps.data_available = FALSE;
    return TRUE;
  }
  return FALSE;
}

bool nps_sensors_sonar_available(void)
{
  if (sensors.sonar.data_available) {
    sensors.sonar.data_available = FALSE;
    return TRUE;
  }
  return FALSE;
}

bool nps_sensors_airspeed_available(void)
{
  if (sensors.airspeed.data_available) {
    sensors.airspeed.data_available = FALSE;
    return TRUE;
  }
  return FALSE;
}

bool nps_sensors_temperature_available(void)
{
  if (sensors.temp.data_available) {
    sensors.temp.data_available = FALSE;
    return TRUE;
  }
  return FALSE;
}

bool nps_sensors_aoa_available(void)
{
  if (sensors.aoa.data_available) {
    sensors.aoa.data_available = FALSE;
    return TRUE;
  }
  return FALSE;
}

bool nps_sensors_sideslip_available(void)
{
  if (sensors.sideslip.data_available) {
    sensors.sideslip.data_available = FALSE;
    return TRUE;
  }
  return FALSE;
}
