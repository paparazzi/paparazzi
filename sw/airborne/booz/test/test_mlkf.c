#include <stdio.h>
#include <string.h>

#include "math/pprz_algebra_double.h"
#include "subsystems/imu.h"
#include "subsystems/ahrs.h"
#include "ahrs/ahrs_mlkf.h"

static void read_data(const char* filename);
static void feed_imu(int i);
static void store_filter_output(int i);
static void dump_output(const char* filename);

#define IN_FILE "../../../simulator/scilab/q6d/data/stop_stop_state_sensors.txt"
#define OUT_FILE "./out.txt"

#define MAX_SAMPLE 15000
struct test_sample {
  double time;
  struct DoubleQuat  quat_true;
  struct DoubleRates omega_true;
  struct DoubleRates gyro;
  struct DoubleVect3 accel;
  struct DoubleVect3 mag;
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


int main(int argc, char** argv) {

  read_data(IN_FILE);

  imu_init();
  ahrs_init();

  for (int i=0; i<nb_samples; i++) {
    feed_imu(i);
    ahrs_propagate();
    ahrs_update_accel();
    ahrs_update_mag();
    store_filter_output(i);
  }
  dump_output(OUT_FILE);

  return 0;
}


static void read_data(const char* filename) {
  FILE* fd = fopen(filename, "r");
  nb_samples = 0;
  int ret = 0;
  do {
    struct test_sample* s = &samples[nb_samples];
    ret = fscanf(fd, "%lf [%lf %lf %lf %lf] [%lf %lf %lf] [%lf %lf %lf] [%lf %lf %lf] [%lf %lf %lf]",
		 &s->time,
		 &s->quat_true.qi, &s->quat_true.qx, &s->quat_true.qy, &s->quat_true.qz,
		 &s->omega_true.p, &s->omega_true.q, &s->omega_true.r,
		 &s->gyro.p, &s->gyro.q, &s->gyro.r,
		 &s->accel.x, &s->accel.y, &s->accel.z,
		 &s->mag.x, &s->mag.y, &s->mag.z );
    nb_samples++;
  }
  while (ret == 17  && nb_samples < MAX_SAMPLE);
  nb_samples--;
  fclose(fd);
  printf("read %d points in file %s\n", nb_samples, filename);
}


static void feed_imu(int i) {
  if (i>0) {
    RATES_COPY(imu.gyro_prev, imu.gyro);
  }
  else {
    RATES_BFP_OF_REAL(imu.gyro_prev, samples[0].gyro);
  }
  RATES_BFP_OF_REAL(imu.gyro, samples[i].gyro);
  ACCELS_BFP_OF_REAL(imu.accel, samples[i].accel);
  MAGS_BFP_OF_REAL(imu.mag, samples[i].mag);
}


static void store_filter_output(int i) {

  QUAT_COPY(output[i].quat_est, ahrs_float.ltp_to_imu_quat);
  RATES_COPY(output[i].bias_est, ahrs_mlkf.gyro_bias);
  RATES_COPY(output[i].rate_est, ahrs_float.imu_rate);
  memcpy(output[i].P, ahrs_mlkf.P, sizeof(ahrs_mlkf.P));

}

static void dump_output(const char* filename) {
  FILE* fd = fopen(filename, "w");
  int i;
  for (i=0; i<nb_samples; i++) {
    fprintf(fd, "%.16f [%.16f %.16f %.16f %.16f] [%.16f %.16f %.16f] [%.16f %.16f %.16f] [%.16f %.16f %.16f %.16f %.16f %.16f]\n",
	    samples[i].time,
	    output[i].quat_est.qi, output[i].quat_est.qx, output[i].quat_est.qy, output[i].quat_est.qz, // quaternion
	    output[i].rate_est.p, output[i].rate_est.q, output[i].rate_est.r,                           // omega
	    output[i].bias_est.p, output[i].bias_est.q, output[i].bias_est.r,                           // bias
            output[i].P[0][0], output[i].P[1][1], output[i].P[2][2],                                    // covariance
            output[i].P[3][3], output[i].P[4][4], output[i].P[5][5] );
  }
  fclose(fd);
  printf("wrote %d points in file %s\n", nb_samples, filename);
}
