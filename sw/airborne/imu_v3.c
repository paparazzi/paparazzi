#include "imu_v3.h"

#include CONFIG
#include "adc.h"

/* accelerometers in arbitrary unit */
float imu_accel[AXIS_NB];
/* gyros in rad/s                   */
float   imu_gyro[AXIS_NB];
/* magnetometer in arbitrary unit   */
int16_t imu_mag[AXIS_NB];
/* battery in volts                 */
float imu_bat;

float imu_gyro_prev[AXIS_NB];


uint16_t imu_accel_raw[AXIS_NB];
uint16_t imu_gyro_raw[AXIS_NB];
int16_t  imu_mag_raw[AXIS_NB];

struct adc_buf buf_ax;
struct adc_buf buf_ay;
struct adc_buf buf_az;
struct adc_buf buf_bat;


#define IMU_DETECT_STILL_LEN 128
bool_t imu_vehicle_still;
float imu_vs_gyro_initial_bias[AXIS_NB];
float imu_vs_gyro_unbiased[AXIS_NB];

uint16_t imu_vs_accel_raw[AXIS_NB][IMU_DETECT_STILL_LEN];
uint32_t imu_vs_accel_raw_sum[AXIS_NB];
uint32_t imu_vs_accel_raw_avg[AXIS_NB];
float    imu_vs_accel_raw_var[AXIS_NB];

uint16_t imu_vs_gyro_raw[AXIS_NB][IMU_DETECT_STILL_LEN];
uint32_t imu_vs_gyro_raw_sum[AXIS_NB];
uint32_t imu_vs_gyro_raw_avg[AXIS_NB];
float    imu_vs_gyro_raw_var[AXIS_NB];

int16_t  imu_vs_mag_raw[AXIS_NB][IMU_DETECT_STILL_LEN];
int32_t  imu_vs_mag_raw_sum[AXIS_NB];
int32_t  imu_vs_mag_raw_avg[AXIS_NB];
float    imu_vs_mag_raw_var[AXIS_NB];

static uint8_t imu_vs_buf_head;
static bool_t imu_vs_buf_filled;

#define IMU_VS_ACCEL_RAW_VAR_MAX 7000.


void imu_init(void) {
  adc_buf_channel(ADC_CHANNEL_AX, &buf_ax, DEFAULT_AV_NB_SAMPLE);
  adc_buf_channel(ADC_CHANNEL_AY, &buf_ay, DEFAULT_AV_NB_SAMPLE);
  adc_buf_channel(ADC_CHANNEL_AZ, &buf_az, DEFAULT_AV_NB_SAMPLE);
  adc_buf_channel(ADC_CHANNEL_BAT, &buf_bat, DEFAULT_AV_NB_SAMPLE);

  imu_vehicle_still = FALSE;
  imu_vs_buf_filled = FALSE;
  imu_vs_buf_head = 0;

  uint8_t i, j;
  for (i=0; i<AXIS_NB; i++) {
    imu_vs_accel_raw_sum[i] = 0;
    imu_vs_gyro_raw_sum[i] = 0;
    imu_vs_mag_raw_sum[i] = 0;
    imu_vs_gyro_initial_bias[i] = 0.;
    for (j=0; j<IMU_DETECT_STILL_LEN; j++) {
      imu_vs_accel_raw[i][j] = 0;
      imu_vs_gyro_raw[i][j] = 0;
      imu_vs_mag_raw[i][j] = 0;
    }
  }
}


void imu_detect_vehicle_still(void) {

  /* update the sliding average of sensors readings */
  imu_vs_buf_head++;
  if (imu_vs_buf_head >= IMU_DETECT_STILL_LEN) {
    imu_vs_buf_filled = TRUE;
    imu_vs_buf_head = 0;
  } 
  
  uint8_t i, j;
  for (i=0; i<AXIS_NB; i++) {
    imu_vs_accel_raw_sum[i] -= imu_vs_accel_raw[i][imu_vs_buf_head];
    imu_vs_accel_raw[i][imu_vs_buf_head] = imu_accel_raw[i];
    imu_vs_accel_raw_sum[i] += imu_accel_raw[i];

    imu_vs_gyro_raw_sum[i] -= imu_vs_gyro_raw[i][imu_vs_buf_head];
    imu_vs_gyro_raw[i][imu_vs_buf_head] = imu_gyro_raw[i];
    imu_vs_gyro_raw_sum[i] += imu_gyro_raw[i];
    
    imu_vs_mag_raw_sum[i] -= imu_vs_mag_raw[i][imu_vs_buf_head];
    imu_vs_mag_raw[i][imu_vs_buf_head] = imu_mag_raw[i];
    imu_vs_mag_raw_sum[i] += imu_mag_raw[i]; 
  }

  /* compute variance */
  if (imu_vs_buf_filled) {
    for (i=0; i<AXIS_NB; i++) {  
      imu_vs_accel_raw_avg[i] = imu_vs_accel_raw_sum[i] / IMU_DETECT_STILL_LEN;
      imu_vs_gyro_raw_avg[i] = imu_vs_gyro_raw_sum[i] / IMU_DETECT_STILL_LEN;
      imu_vs_mag_raw_avg[i] = imu_vs_mag_raw_sum[i] / IMU_DETECT_STILL_LEN;

      imu_vs_accel_raw_var[i] = 0.;
      imu_vs_gyro_raw_var[i] = 0.;
      imu_vs_mag_raw_var[i] = 0.;

      for (j=0; j<IMU_DETECT_STILL_LEN; j++) {
	int32_t diff = imu_vs_gyro_raw[i][j] - imu_vs_gyro_raw_avg[i];
	imu_vs_gyro_raw_var[i] += (float)(diff*diff);
	diff = imu_vs_accel_raw[i][j] - imu_vs_accel_raw_avg[i];
	imu_vs_accel_raw_var[i] += (float)(diff*diff);
	diff = imu_vs_mag_raw[i][j] - imu_vs_mag_raw_avg[i];
	imu_vs_mag_raw_var[i] += (float)(diff*diff);
      }
      imu_vs_gyro_raw_var[i] /= (float)IMU_DETECT_STILL_LEN;
      imu_vs_accel_raw_var[i] /= (float)IMU_DETECT_STILL_LEN;
      imu_vs_mag_raw_var[i] /= (float)IMU_DETECT_STILL_LEN;
    }
    //    return;
    /* check vehicle still */
    if (imu_vs_accel_raw_var[0] < IMU_VS_ACCEL_RAW_VAR_MAX &&
	imu_vs_accel_raw_var[1] < IMU_VS_ACCEL_RAW_VAR_MAX &&
	imu_vs_accel_raw_var[2] < IMU_VS_ACCEL_RAW_VAR_MAX )
      imu_vehicle_still = TRUE;
  }

}



