


#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>

#include "std.h"

#include "math/pprz_algebra_float.h"
#include "math/pprz_algebra_double.h"
#include "math/pprz_algebra_int.h"
#include "pprz_algebra_print.h"

#include "subsystems/ahrs.h"
#include "subsystems/ahrs/ahrs_aligner.h"
#include "modules/imu/imu.h"

#define AHRS_TYPE_FLG 0
#define AHRS_TYPE_FCR 1
#define AHRS_TYPE_ICE 2

#if   defined AHRS_TYPE && AHRS_TYPE == AHRS_TYPE_FLQ
#include "subsystems/ahrs/ahrs_float_lkf_quat.h"
#define OUT_FILE "./out_flq.txt"
#elif defined AHRS_TYPE && AHRS_TYPE == AHRS_TYPE_FCR
#include "subsystems/ahrs/ahrs_float_cmpl.h"
#define OUT_FILE "./out_fcr.txt"
#elif defined AHRS_TYPE && AHRS_TYPE == AHRS_TYPE_ICE
#include "subsystems/ahrs/ahrs_int_cmpl_euler.h"
#define OUT_FILE "./out_ice.txt"
#endif


#define MAX_SAMPLE 1000000
//#define MAX_SAMPLE 5000
struct test_sample {
  double time;
  uint8_t flag;
  struct DoubleQuat  quat_true;
  struct DoubleRates omega_true;
  struct DoubleRates gyro;
  struct DoubleVect3 accel;
  struct DoubleVect3 mag;
  struct DoubleVect3 gps_pecef;
  struct DoubleVect3 gps_vecef;
  double baro;
};
static struct test_sample samples[MAX_SAMPLE];
static int nb_samples;

struct test_output {
  struct FloatQuat  quat_est;
  struct FloatRates bias_est;
  struct FloatRates rate_est;
  float P[6][6];
};
static struct test_output output[MAX_SAMPLE];



//#define IN_FILE  "../../fms/libeknav/flight9_spin_circle.ascii"
//#define IN_FILE  "../../fms/libeknav/flight9_2climbs.ascii"
#define IN_FILE  "/home/poine/work/savannah/paparazzi3.broken_stm32_deletion/trunk/sw/simulator/scilab/q6d/data/stop_stop_sensors.txt"


static void read_ascii_flight_log(const char *filename);
static void feed_imu(int i);
static void store_filter_output(int i);
static void dump_output(const char *filename);

/* from fms_autopilot_msg.h */
#define VI_IMU_DATA_VALID      0
#define VI_MAG_DATA_VALID      1
#define IMU_AVAILABLE(_flag) (_flag & (1<<VI_IMU_DATA_VALID))
#define MAG_AVAILABLE(_flag) (_flag & (1<<VI_MAG_DATA_VALID))

int main(int argc, char **argv)
{

  read_ascii_flight_log(IN_FILE);

  imu_init();
  ahrs_init();

  for (int i = 0; i < nb_samples; i++) {
    feed_imu(i);
    if (ahrs.status == AHRS_UNINIT) {
      ahrs_aligner_run();
      if (ahrs_aligner.status == AHRS_ALIGNER_LOCKED) {
        ahrs_align();
      }
    } else {
      ahrs_propagate((1. / AHRS_PROPAGATE_FREQUENCY));
#ifdef ENABLE_MAG_UPDATE
      if (MAG_AVAILABLE(samples[i].flag)) {
        ahrs_update_mag();
      }
#endif
#ifdef ENABLE_ACCEL_UPDATE
      if (IMU_AVAILABLE(samples[i].flag) && (!MAG_AVAILABLE(samples[i].flag))) {
        ahrs_update_accel();
      }
#endif
    }
    store_filter_output(i);
  }

  dump_output(OUT_FILE);

}

/*
 * Read an ascii flight log
 */
static void read_ascii_flight_log(const char *filename)
{
  FILE *fd = fopen(filename, "r");
  nb_samples = 0;
  int ret = 0;
  do {
    struct test_sample *s = &samples[nb_samples];
    ret = fscanf(fd, "%lf\t%hhu\t%lf %lf %lf\t%lf %lf %lf\t%lf %lf %lf\t%lf %lf %lf\t%lf %lf %lf\t%lf",
                 &s->time, &s->flag,
                 &s->gyro.p, &s->gyro.q, &s->gyro.r,
                 &s->accel.x, &s->accel.y, &s->accel.z,
                 &s->mag.x, &s->mag.y, &s->mag.z,
                 &s->gps_pecef.x, &s->gps_pecef.y, &s->gps_pecef.z,
                 &s->gps_vecef.x, &s->gps_vecef.y, &s->gps_vecef.z,
                 &s->baro);
    nb_samples++;
  } while (ret == 18  && nb_samples < MAX_SAMPLE);
  nb_samples--;
  fclose(fd);
  printf("read %d points in file %s\n", nb_samples, filename);
}


/*
 *
 */
static void feed_imu(int i)
{
  if (i > 0) {
    RATES_COPY(imu.gyro_prev, imu.gyro);
  } else {
    RATES_BFP_OF_REAL(imu.gyro_prev, samples[0].gyro);
  }
  RATES_BFP_OF_REAL(imu.gyro, samples[i].gyro);
  ACCELS_BFP_OF_REAL(imu.accel, samples[i].accel);
  MAGS_BFP_OF_REAL(imu.mag, samples[i].mag);
}

/*
 *
 */
#if   defined AHRS_TYPE && AHRS_TYPE == AHRS_TYPE_FLQ
static void store_filter_output(int i)
{
#ifdef OUTPUT_IN_BODY_FRAME
  QUAT_COPY(output[i].quat_est, ahrs_impl.ltp_to_body_quat);
  RATES_COPY(output[i].rate_est, ahrs_impl.body_rate);
#else
  QUAT_COPY(output[i].quat_est, ahrs_impl.ltp_to_imu_quat);
  RATES_COPY(output[i].rate_est, ahrs_impl.imu_rate);
#endif /* OUTPUT_IN_BODY_FRAME */
  RATES_COPY(output[i].bias_est, ahrs_impl.gyro_bias);
  memcpy(output[i].P, ahrs_impl.P, sizeof(ahrs_impl.P));
}
#elif defined AHRS_TYPE && AHRS_TYPE == AHRS_TYPE_FCR
static void store_filter_output(int i)
{
#ifdef OUTPUT_IN_BODY_FRAME
  QUAT_COPY(output[i].quat_est, ahrs_impl.ltp_to_body_quat);
  RATES_COPY(output[i].rate_est, ahrs_impl.body_rate);
#else
  QUAT_COPY(output[i].quat_est, ahrs_impl.ltp_to_imu_quat);
  RATES_COPY(output[i].rate_est, ahrs_impl.imu_rate);
#endif /* OUTPUT_IN_BODY_FRAME */
  RATES_COPY(output[i].bias_est, ahrs_impl.gyro_bias);
  //  memcpy(output[i].P, ahrs_impl.P, sizeof(ahrs_impl.P));
}
#elif defined AHRS_TYPE && AHRS_TYPE == AHRS_TYPE_ICE
static void store_filter_output(int i)
{
#ifdef OUTPUT_IN_BODY_FRAME
  QUAT_FLOAT_OF_BFP(output[i].quat_est, ahrs_impl.ltp_to_body_quat);
  RATES_FLOAT_OF_BFP(output[i].rate_est, ahrs_impl.body_rate);
#else
  struct FloatEulers eul_f;
  EULERS_FLOAT_OF_BFP(eul_f, ahrs_impl.ltp_to_imu_euler);
  float_quat_of_eulers(&output[i].quat_est, &eul_f);
  RATES_FLOAT_OF_BFP(output[i].rate_est, ahrs_impl.imu_rate);
#endif /* OUTPUT_IN_BODY_FRAME */
  RATES_ASSIGN(output[i].bias_est, 0., 0., 0.);
  //  memset(output[i].P, ahrs_impl.P, sizeof(ahrs_impl.P));
}
#endif

/*
 *
 */
static void dump_output(const char *filename)
{
  FILE *fd = fopen(filename, "w");
  int i;
  for (i = 0; i < nb_samples; i++) {
    fprintf(fd,
            "%.16f\t%.16f %.16f %.16f %.16f\t%.16f %.16f %.16f\t%.16f %.16f %.16f\t%.16f %.16f %.16f %.16f %.16f %.16f\n",
            samples[i].time,
            output[i].quat_est.qi, output[i].quat_est.qx, output[i].quat_est.qy, output[i].quat_est.qz, // quaternion
            output[i].rate_est.p, output[i].rate_est.q, output[i].rate_est.r,                           // omega
            output[i].bias_est.p, output[i].bias_est.q, output[i].bias_est.r,                           // bias
            output[i].P[0][0], output[i].P[1][1], output[i].P[2][2],                                    // covariance
            output[i].P[3][3], output[i].P[4][4], output[i].P[5][5]);
  }
  fclose(fd);
  printf("wrote %d points in file %s\n", nb_samples, filename);
}
