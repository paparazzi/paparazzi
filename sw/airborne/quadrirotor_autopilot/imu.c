#include <inttypes.h>

#include "imu.h"
#include "airframe.h"
#include "adc.h"

struct ImuSample imu_sample;

//static struct adc_buf buf_gyro_roll;
//static struct adc_buf buf_accel_y;
//static struct adc_buf buf_accel_z;


/* carte sur le coté Y<->Z */

#define GYRO_X_NEUTRAL  425
#define GYRO_X_GAIN     0.0170

#define GYRO_Y_NEUTRAL  463
#define GYRO_Y_GAIN     -0.0179

#define GYRO_Z_NEUTRAL  460
#define GYRO_Z_GAIN     0.0180

#define ACCEL_X_MIN 272
#define ACCEL_X_MAX 689

#define ACCEL_X_NEUTRAL ((ACCEL_X_MIN + ACCEL_X_MAX)/2)
#define ACCEL_X_GAIN    (2 * 9.81 / (ACCEL_X_MAX - ACCEL_X_MIN))

#define ACCEL_Y_MIN 725
#define ACCEL_Y_MAX 272

#define ACCEL_Y_NEUTRAL ((ACCEL_Y_MIN + ACCEL_Y_MAX)/2)
#define ACCEL_Y_GAIN    (2 * 9.81 / (ACCEL_Y_MAX - ACCEL_Y_MIN))

#define ACCEL_Z_MIN 635
#define ACCEL_Z_MAX 226

#define ACCEL_Z_NEUTRAL ((ACCEL_Z_MIN + ACCEL_Z_MAX)/2)
#define ACCEL_Z_GAIN    (2 * 9.81 / (ACCEL_Z_MAX - ACCEL_Z_MIN))

/* carte debout
Gy :
 - 0: 460
 - 16 T/mn: 460
 - 33 T/mn : 370
 - broken
Gz:
 - 0: 463
 - 16: 363
 - 33: 270
 - gain 1.79e-2


Gx:
 - 0: 425
 - 16: 325
 - 33: 220
 - gain 1.69e-2
 */

void imu_init( void ) {
  //  adc_buf_channel(ADC_CHANNEL_GYRO_X, &buf_gyro_roll);
  //  adc_buf_channel(ADC_CHANNEL_ACCEL_Y, &buf_accel_y);
  //  adc_buf_channel(ADC_CHANNEL_ACCEL_Z, &buf_accel_z);
}

void imu_update( void ) {
  imu_sample.gyro_x = (float)((int16_t)adc_samples[ADC_CHANNEL_GYRO_X] - GYRO_X_NEUTRAL) * GYRO_X_GAIN;
  imu_sample.gyro_y = (float)((int16_t)adc_samples[ADC_CHANNEL_GYRO_Y] - GYRO_Y_NEUTRAL) * GYRO_Y_GAIN;
  imu_sample.gyro_z = (float)((int16_t)adc_samples[ADC_CHANNEL_GYRO_Z] - GYRO_Z_NEUTRAL) * GYRO_Z_GAIN;
  imu_sample.accel_x = (float)((int16_t)adc_samples[ADC_CHANNEL_ACCEL_X] - ACCEL_X_NEUTRAL) * ACCEL_X_GAIN;
  imu_sample.accel_y = (float)((int16_t)adc_samples[ADC_CHANNEL_ACCEL_Y] - ACCEL_Y_NEUTRAL) * ACCEL_Y_GAIN;
  imu_sample.accel_z = (float)((int16_t)adc_samples[ADC_CHANNEL_ACCEL_Z] - ACCEL_Z_NEUTRAL) * ACCEL_Z_GAIN;
}
