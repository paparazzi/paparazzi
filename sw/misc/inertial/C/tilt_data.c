#include "tilt_data.h"

#include "random.h"

#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <stdio.h>

#include <glib.h>

#define DT 0.01
#define T_END 90.
#define NB_POINTS (int)(T_END/DT)
#define T0 10.
#define T1 11.
#define T2 12.
#define DELTA_T1 (T1 - T0)
#define DELTA_T2 (T2 - T1)
#define MAX_RATE 0.75

#define NOISE_GYRO  0.05
#define BIAS_GYRO   0.02
#define DBIAS_GYRO  0.005
#define NOISE_ACCEL 1.
#define G 9.81

struct tilt_data* tilt_data_gen(void) {
  struct tilt_data* td = tilt_data_new(NB_POINTS, DT);
  int i;
  for (i=0; i< NB_POINTS; i++) {
    td->t[i] = i * DT;
    if ((td->t[i] < T0) | (td->t[i] > T2)) {
      td->rate[i] = 0.;
      td->angle[i] = 0.;
    }
    else {
      if ( td->t[i] < T1)
	td->rate[i] = MAX_RATE * ( 1. - cos((td->t[i] - T0)/DELTA_T1*2*M_PI));
      else
	td->rate[i] = MAX_RATE * ( -1. + cos((td->t[i] - T1)/DELTA_T2*2*M_PI));
      td->angle[i] = td->angle[i-1] + td->rate[i] * td->dt;
    }
    td->bias[i] = td->t[i] < 30. ? BIAS_GYRO * ( 1. + td->t[i] * DBIAS_GYRO) :
      BIAS_GYRO * ( 1. + 30. * DBIAS_GYRO);
  }

  for (i=0; i< NB_POINTS; i++) {
    td->gyro[i] = td->rate[i] + td->bias[i] + NOISE_GYRO * ukf_filter_rand_normal();
    td->ay[i] = G * sin(td->angle[i]) + NOISE_ACCEL * ukf_filter_rand_normal();
    td->az[i] = G * cos(td->angle[i]) + NOISE_ACCEL * ukf_filter_rand_normal();
    td->m_angle[i] = atan2(td->ay[i], td->az[i]);
  }

  return td;
}

void tilt_data_save_state(struct tilt_data* td, int idx, double* X, double* P) {
  td->est_angle[idx] = X[0];
  td->est_bias[idx] = X[1];
  /* FIXME : is this correct ?? not sure about the row/col order */
  td->P00[idx] = P[0];
  td->P01[idx] = P[1];
  td->P10[idx] = P[2];
  td->P11[idx] = P[3];
}

struct tilt_data* tilt_data_read_log(const char* filename) {
  GError* _err = NULL;
  GIOChannel* in_c = g_io_channel_new_file(filename, "r", &_err);
  if (_err) {
    g_free(_err);
    return NULL;
  }

  GArray* ga_gyro = g_array_new(FALSE, FALSE, sizeof(double));
  GArray* ga_ay = g_array_new(FALSE, FALSE, sizeof(double));
  GArray* ga_az = g_array_new(FALSE, FALSE, sizeof(double));

  GString *str = g_string_sized_new(256);
  GIOStatus st;
  do {
    double gx, gy, gz, ax, ay, az;
    st = g_io_channel_read_line_string(in_c, str, NULL, &_err);
    if (g_str_has_prefix(str->str, "IMU_GYRO")) {
      if (sscanf(str->str, "IMU_GYRO %lf %lf %lf", &gx, &gy, &gz) == 3) {
	g_array_append_val(ga_gyro, gx);
      }
    }
    else if (g_str_has_prefix(str->str, "IMU_ACCEL")) {
      if (sscanf(str->str, "IMU_ACCEL %lf %lf %lf", &ax, &ay, &az) == 3) {
	g_array_append_val(ga_ay, ay);
	g_array_append_val(ga_az, az);
      }
    }
  }
  while (st != G_IO_STATUS_EOF);
  g_string_free(str, TRUE);
  g_free(in_c);

  struct tilt_data* td = tilt_data_new(ga_gyro->len, 0.015625);
  int i;
  for (i=0; i<ga_gyro->len; i++) {
    double* ggx = &g_array_index(ga_gyro, double, i);
    td->gyro[i] = *ggx;
    double* gay = &g_array_index(ga_ay, double, i);
    td->ay[i] = *gay;
    double* gaz = &g_array_index(ga_az, double, i);
    td->az[i] = *gaz;
    td->m_angle[i] = atan2(td->ay[i], td->az[i]);
  }

  return td;
}

struct tilt_data* tilt_data_new(int len, double dt) {
  struct tilt_data* td = malloc(sizeof(struct tilt_data));
  td->dt = dt;
  td->nb_samples = len;

  td->t = malloc(td->nb_samples*sizeof(double));
  td->rate = malloc(td->nb_samples*sizeof(double));
  td->angle = malloc(td->nb_samples*sizeof(double));
  td->bias = malloc(td->nb_samples*sizeof(double));

  td->gyro = malloc(td->nb_samples*sizeof(double));
  td->ay = malloc(td->nb_samples*sizeof(double));
  td->az = malloc(td->nb_samples*sizeof(double));
  td->m_angle = malloc(td->nb_samples*sizeof(double));

  td->est_angle = malloc(td->nb_samples*sizeof(double));
  td->est_bias = malloc(td->nb_samples*sizeof(double));

  td->P00 = malloc(td->nb_samples*sizeof(double));
  td->P01 = malloc(td->nb_samples*sizeof(double));
  td->P10 = malloc(td->nb_samples*sizeof(double));
  td->P11 = malloc(td->nb_samples*sizeof(double));

  int i;
  for (i=0; i< td->nb_samples; i++) {
    td->t[i] = i * td->dt;
    td->rate[i] = 0.;
    td->angle[i] = 0.;
    td->bias[i] = 0.;

    td->gyro[i] = 0.;
    td->ay[i] = 0.;
    td->az[i] = 0.;
    td->m_angle[i] = 0.;

    td->est_angle[i] = 0.;
    td->est_bias[i] = 0.;
    td->P00[i] = 0.;
    td->P01[i] = 0.;
    td->P10[i] = 0.;
    td->P11[i] = 0.;
  }
  return td;
}
