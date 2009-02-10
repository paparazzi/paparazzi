#include "booz2_filter_aligner.h"

#include "booz2_imu.h"
#include "led.h"

uint8_t booz2_filter_aligner_status;

struct booz_ivect booz2_filter_aligner_lp_gyro;
struct booz_ivect booz2_filter_aligner_lp_accel;
struct booz_ivect booz2_filter_aligner_lp_mag;
int32_t booz2_filter_aligner_noise;
int32_t booz2_filter_aligner_low_noise_cnt;

#define SAMPLES_NB 512
static struct booz_ivect gyro_sum;
static struct booz_ivect accel_sum;
static struct booz_ivect mag_sum;
static int32_t ref_sensor_samples[SAMPLES_NB];
static uint32_t samples_idx;

void booz2_filter_aligner_init(void) {

  booz2_filter_aligner_status = BOOZ2_FILTER_ALIGNER_RUNNING;
  BOOZ_IVECT_ZERO(gyro_sum);
  BOOZ_IVECT_ZERO(accel_sum);
  BOOZ_IVECT_ZERO(mag_sum);
  samples_idx = 0;
  booz2_filter_aligner_noise = 0;
  booz2_filter_aligner_low_noise_cnt = 0;
}

#define LOW_NOISE_THRESHOLD 90000
#define LOW_NOISE_TIME          5

void booz2_filter_aligner_run(void) {

  BOOZ_IVECT_SUM(gyro_sum,  gyro_sum,  booz2_imu_gyro);
  BOOZ_IVECT_SUM(accel_sum, accel_sum, booz2_imu_accel);
  BOOZ_IVECT_SUM(mag_sum,   mag_sum   ,booz2_imu_mag);

  ref_sensor_samples[samples_idx] = booz2_imu_accel.z;
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
    
    booz2_filter_aligner_noise = 0;
    int i;
    for (i=0; i<SAMPLES_NB; i++) {
      int32_t diff = ref_sensor_samples[i] - avg_ref_sensor;
      booz2_filter_aligner_noise += abs(diff);
    }

    BOOZ_IVECT_SDIV_ACC(booz2_filter_aligner_lp_gyro,  gyro_sum,  SAMPLES_NB);
    BOOZ_IVECT_SDIV_ACC(booz2_filter_aligner_lp_accel, accel_sum, SAMPLES_NB);
    BOOZ_IVECT_SDIV_ACC(booz2_filter_aligner_lp_mag,   mag_sum,   SAMPLES_NB);

    BOOZ_IVECT_ZERO(gyro_sum);
    BOOZ_IVECT_ZERO(accel_sum);
    BOOZ_IVECT_ZERO(mag_sum);
    samples_idx = 0;

    if (booz2_filter_aligner_noise < LOW_NOISE_THRESHOLD)
      booz2_filter_aligner_low_noise_cnt++;
    else
      if ( booz2_filter_aligner_low_noise_cnt > 0)
	booz2_filter_aligner_low_noise_cnt--;

    if (booz2_filter_aligner_low_noise_cnt > LOW_NOISE_TIME) {
      booz2_filter_aligner_status = BOOZ2_FILTER_ALIGNER_LOCKED;
#ifdef FILTER_ALIGNER_LED
      LED_ON(FILTER_ALIGNER_LED);
#endif
    }
  }

}

