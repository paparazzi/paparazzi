#include "booz_ahrs_aligner.h"

#include "booz2_imu.h"
#include "led.h"

struct BoozAhrsAligner booz_ahrs_aligner;

#define SAMPLES_NB 512
static struct Int32Rates gyro_sum;
static struct Int32Vect3 accel_sum;
static struct Int32Vect3 mag_sum;
static int32_t ref_sensor_samples[SAMPLES_NB];
static uint32_t samples_idx;

void booz_ahrs_aligner_init(void) {
  
  booz_ahrs_aligner.status = BOOZ_AHRS_ALIGNER_RUNNING;
  INT_RATES_ZERO(gyro_sum);
  INT_VECT3_ZERO(accel_sum);
  INT_VECT3_ZERO(mag_sum);
  samples_idx = 0;
  booz_ahrs_aligner.noise = 0;
  booz_ahrs_aligner.low_noise_cnt = 0;

}

#define LOW_NOISE_THRESHOLD 90000
#define LOW_NOISE_TIME          5

void booz_ahrs_aligner_run(void) {

  RATES_ADD(gyro_sum,  booz_imu.gyro);
  VECT3_ADD(accel_sum, booz_imu.accel);
  VECT3_ADD(mag_sum,   booz_imu.mag);

  ref_sensor_samples[samples_idx] = booz_imu.accel.z;
  samples_idx++;

#ifdef FILTER_ALIGNER_LED
  RunOnceEvery(50, {LED_TOGGLE(FILTER_ALIGNER_LED);});
#endif

  if (samples_idx >= SAMPLES_NB) {
    int32_t avg_ref_sensor = accel_sum.z;
    if ( avg_ref_sensor >= 0)
      avg_ref_sensor += SAMPLES_NB / 2;
    else
      avg_ref_sensor -= SAMPLES_NB / 2;
    avg_ref_sensor /= SAMPLES_NB;
    
    booz_ahrs_aligner.noise = 0;
    int i;
    for (i=0; i<SAMPLES_NB; i++) {
      int32_t diff = ref_sensor_samples[i] - avg_ref_sensor;
      booz_ahrs_aligner.noise += abs(diff);
    }

    RATES_SDIV(booz_ahrs_aligner.lp_gyro,  gyro_sum,  SAMPLES_NB);
    VECT3_SDIV(booz_ahrs_aligner.lp_accel, accel_sum, SAMPLES_NB);
    VECT3_SDIV(booz_ahrs_aligner.lp_mag,   mag_sum,   SAMPLES_NB);

    INT_RATES_ZERO(gyro_sum);
    INT_VECT3_ZERO(accel_sum);
    INT_VECT3_ZERO(mag_sum);
    samples_idx = 0;

    if (booz_ahrs_aligner.noise < LOW_NOISE_THRESHOLD)
      booz_ahrs_aligner.low_noise_cnt++;
    else
      if ( booz_ahrs_aligner.low_noise_cnt > 0)
	booz_ahrs_aligner.low_noise_cnt--;
    
    if (booz_ahrs_aligner.low_noise_cnt > LOW_NOISE_TIME) {
      booz_ahrs_aligner.status = BOOZ_AHRS_ALIGNER_LOCKED;
#ifdef FILTER_ALIGNER_LED
      LED_ON(FILTER_ALIGNER_LED);
#endif
    }
  }

}

