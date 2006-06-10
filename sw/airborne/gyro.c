#include CONFIG
#include "gyro.h"
#include "std.h"
#include "adc.h"
#include "airframe.h"

int16_t roll_rate_adc;
float roll_rate;

static struct adc_buf buf_roll;

#if defined SPARK_FUN
#define RadiansOfADC(_adc) RadOfDeg((_adc/3.41))
static struct adc_buf buf_temp;
float temp_comp;
#elif defined IDC300
#define RadiansOfADC(_adc) RadOfDeg((_adc/3.41))
static struct adc_buf buf_pitch;
float pitch_rate;
#endif

void gyro_init( void) {
  adc_buf_channel(ADC_CHANNEL_GYRO_ROLL, &buf_roll, ADC_CHANNEL_GYRO_NB_SAMPLES);

#if defined SPARK_FUN
  adc_buf_channel(ADC_CHANNEL_GYRO_TEMP, &buf_temp, ADC_CHANNEL_GYRO_NB_SAMPLES);
#elif defined IDC300
  adc_buf_channel(ADC_CHANNEL_GYRO_PITCH, &buf_pitch, ADC_CHANNEL_GYRO_NB_SAMPLES);
#endif
}



void gyro_update( void ) {
#ifdef SPARK_FUN
  temp_comp = buf_temp.sum/buf_temp.av_nb_sample - GYRO_ADC_TEMP_NEUTRAL;
  
  roll_rate_adc = (buf_roll.sum/buf_roll.av_nb_sample) - (GYRO_ADC_ROLL_NEUTRAL+(GYRO_ADC_TEMP_SLOPE*temp_comp)); 
#elif defined IDC300
  pitch_rate = buf_pitch.sum/buf_pitch.av_nb_sample - GYRO_ADC_PITCH_NEUTRAL;
  pitch_rate = RadiansOfADC(pitch_rate);
  roll_rate_adc = buf_roll.sum/buf_roll.av_nb_sample - GYRO_ADC_ROLL_NEUTRAL;
#endif
  roll_rate = GYRO_ADC_ROLL_COEF * RadiansOfADC(roll_rate_adc);
}
