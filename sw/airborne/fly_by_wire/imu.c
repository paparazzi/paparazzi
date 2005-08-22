#include "imu.h"

#include "airframe.h"

int16_t roll_dot, pitch_dot, yaw_dot;

#ifdef IMU_TYPE_3DMG
#include "3dmg.h"
int16_t roll, pitch, yaw;

void imu_init ( void ) {
  _3dmg_set_continuous_mode();
}

void imu_update ( void ) {
  roll_dot = _3dmg_roll_dot;
  pitch_dot = _3dmg_pitch_dot;
  yaw_dot = _3dmg_yaw_dot;

  roll = _3dmg_roll;
  pitch = _3dmg_pitch;
  yaw = _3dmg_yaw;
}

void imu_capture_neutral ( void ) {
  _3dmg_capture_neutral();
}

#endif

#ifdef IMU_TYPE_ANALOG
#include "adc_fbw.h"

struct adc_buf buf_roll_dot;
struct adc_buf buf_pitch_dot;
struct adc_buf buf_yaw_dot;

uint16_t raw_roll_dot  = 512*AV_NB_SAMPLE;
uint16_t raw_pitch_dot = 512*AV_NB_SAMPLE;
uint16_t raw_yaw_dot   = 512*AV_NB_SAMPLE;

uint16_t raw_roll_dot_neutral  = 512*AV_NB_SAMPLE;
uint16_t raw_pitch_dot_neutral = 512*AV_NB_SAMPLE;
uint16_t raw_yaw_dot_neutral   = 512*AV_NB_SAMPLE;

void imu_init ( void ) {
  adc_buf_channel(IMU_ADC_ROLL_DOT, &buf_roll_dot);
  adc_buf_channel(IMU_ADC_PITCH_DOT, &buf_pitch_dot);
  adc_buf_channel(IMU_ADC_YAW_DOT, &buf_yaw_dot);
}

void imu_update ( void ) {
  raw_roll_dot = buf_roll_dot.sum;
  raw_pitch_dot = buf_pitch_dot.sum;
  raw_yaw_dot = buf_yaw_dot.sum;
  roll_dot = (int16_t)(raw_roll_dot - raw_roll_dot_neutral) / AV_NB_SAMPLE;
  pitch_dot = (int16_t)(raw_pitch_dot - raw_pitch_dot_neutral) / AV_NB_SAMPLE;
  yaw_dot = (int16_t)(raw_yaw_dot - raw_yaw_dot_neutral) / AV_NB_SAMPLE;;
}

void imu_capture_neutral ( void ) {
  raw_roll_dot_neutral = raw_roll_dot;
  raw_pitch_dot_neutral = raw_pitch_dot;
  raw_yaw_dot_neutral = raw_yaw_dot;
}

#endif
