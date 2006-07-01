#include "imu_v3.h"

#include CONFIG
#include "adc.h"

/* accelerometers in arbitrary unit */
int16_t imu_accel[AXIS_NB];
/* gyros in rad/s                   */
float   imu_gyro[AXIS_NB];
/* magnetometer in arbitrary unit   */
int16_t imu_mag[AXIS_NB];
/* battery in volts                 */
float imu_bat;

struct adc_buf buf_ax;
/* struct adc_buf buf_ay; */
struct adc_buf buf_az;
struct adc_buf buf_bat;

void imu_init(void) {
  adc_buf_channel(ADC_CHANNEL_AX, &buf_ax, DEFAULT_AV_NB_SAMPLE);
  adc_buf_channel(ADC_CHANNEL_AY, &buf_ay, DEFAULT_AV_NB_SAMPLE);
  adc_buf_channel(ADC_CHANNEL_AZ, &buf_az, DEFAULT_AV_NB_SAMPLE);
  adc_buf_channel(ADC_CHANNEL_BAT, &buf_bat, DEFAULT_AV_NB_SAMPLE);
}
