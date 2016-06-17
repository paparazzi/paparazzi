#include "ahrs_data.h"

#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <stdio.h>
#include <glib.h>

#include "random.h"
#include "ahrs_utils.h"

struct ahrs_data* ahrs_data_new(int len, double dt) {

  struct ahrs_data* ad = malloc(sizeof(struct ahrs_data));
  ad->dt = dt;
  ad->nb_samples = len;

  ad->t = malloc(ad->nb_samples*sizeof(double));

  ad->phi   = malloc(ad->nb_samples*sizeof(double));
  ad->theta = malloc(ad->nb_samples*sizeof(double));
  ad->psi   = malloc(ad->nb_samples*sizeof(double));

  ad->bias_p = malloc(ad->nb_samples*sizeof(double));
  ad->bias_q = malloc(ad->nb_samples*sizeof(double));
  ad->bias_r = malloc(ad->nb_samples*sizeof(double));


  ad->gyro_p = malloc(ad->nb_samples*sizeof(double));
  ad->gyro_q = malloc(ad->nb_samples*sizeof(double));
  ad->gyro_r = malloc(ad->nb_samples*sizeof(double));

  ad->accel_x = malloc(ad->nb_samples*sizeof(double));
  ad->accel_y = malloc(ad->nb_samples*sizeof(double));
  ad->accel_z = malloc(ad->nb_samples*sizeof(double));

  ad->mag_x = malloc(ad->nb_samples*sizeof(double));
  ad->mag_y = malloc(ad->nb_samples*sizeof(double));
  ad->mag_z = malloc(ad->nb_samples*sizeof(double));

  ad->m_phi = malloc(ad->nb_samples*sizeof(double));;
  ad->m_theta = malloc(ad->nb_samples*sizeof(double));;
  ad->m_psi = malloc(ad->nb_samples*sizeof(double));;

  ad->est_phi   = malloc(ad->nb_samples*sizeof(double));
  ad->est_theta = malloc(ad->nb_samples*sizeof(double));
  ad->est_psi   = malloc(ad->nb_samples*sizeof(double));

  ad->est_bias_p = malloc(ad->nb_samples*sizeof(double));
  ad->est_bias_q = malloc(ad->nb_samples*sizeof(double));
  ad->est_bias_r = malloc(ad->nb_samples*sizeof(double));

  ad->P11 = malloc(ad->nb_samples*sizeof(double));
  ad->P22 = malloc(ad->nb_samples*sizeof(double));
  ad->P33 = malloc(ad->nb_samples*sizeof(double));
  ad->P44 = malloc(ad->nb_samples*sizeof(double));
  ad->P55 = malloc(ad->nb_samples*sizeof(double));
  ad->P66 = malloc(ad->nb_samples*sizeof(double));
  ad->P77 = malloc(ad->nb_samples*sizeof(double));

  return ad;
}


struct ahrs_data* ahrs_data_read_log(const char* filename) {
  GError* _err = NULL;
  GIOChannel* in_c = g_io_channel_new_file(filename, "r", &_err);
  if (_err) {
    g_free(_err);
    return NULL;
  }

  GArray* ga_gx = g_array_new(FALSE, FALSE, sizeof(double));
  GArray* ga_gy = g_array_new(FALSE, FALSE, sizeof(double));
  GArray* ga_gz = g_array_new(FALSE, FALSE, sizeof(double));
  GArray* ga_ax = g_array_new(FALSE, FALSE, sizeof(double));
  GArray* ga_ay = g_array_new(FALSE, FALSE, sizeof(double));
  GArray* ga_az = g_array_new(FALSE, FALSE, sizeof(double));
  GArray* ga_mx = g_array_new(FALSE, FALSE, sizeof(double));
  GArray* ga_my = g_array_new(FALSE, FALSE, sizeof(double));
  GArray* ga_mz = g_array_new(FALSE, FALSE, sizeof(double));

  GString *str = g_string_sized_new(256);
  GIOStatus st;
  do {
    double x, y, z;
    st = g_io_channel_read_line_string(in_c, str, NULL, &_err);
    if (g_str_has_prefix(str->str, "IMU_GYRO")) {
      if (sscanf(str->str, "IMU_GYRO %lf %lf %lf", &x, &y, &z) == 3) {
	g_array_append_val(ga_gx, x);
	g_array_append_val(ga_gy, y);
	g_array_append_val(ga_gz, z);
      }
    }
    else if (g_str_has_prefix(str->str, "IMU_ACCEL")) {
      if (sscanf(str->str, "IMU_ACCEL %lf %lf %lf", &x, &y, &z) == 3) {
	g_array_append_val(ga_ax, x);
	g_array_append_val(ga_ay, y);
	g_array_append_val(ga_az, z);
      }
    }
    else if (g_str_has_prefix(str->str, "IMU_MAG")) {
      if (sscanf(str->str, "IMU_MAG %lf %lf %lf", &x, &y, &z) == 3) {
	g_array_append_val(ga_mx, x);
	g_array_append_val(ga_my, y);
	g_array_append_val(ga_mz, z);
      }
    }
  }
  while (st != G_IO_STATUS_EOF);
  g_string_free(str, TRUE);
  g_free(in_c);

  struct ahrs_data* ad = ahrs_data_new(ga_gx->len, 0.015625);
  int i;
  for (i=0; i<ga_gx->len; i++) {
    ad->t[i] = (double)i * ad->dt;

    double* ggx = &g_array_index(ga_gx, double, i);
    ad->gyro_p[i] = *ggx;
    double* ggy = &g_array_index(ga_gy, double, i);
    ad->gyro_q[i] = *ggy;
    double* ggz = &g_array_index(ga_gz, double, i);
    ad->gyro_r[i] = *ggz;

    double* gax = &g_array_index(ga_ax, double, i);
    ad->accel_x[i] = *gax;
    double* gay = &g_array_index(ga_ay, double, i);
    ad->accel_y[i] = *gay;
    double* gaz = &g_array_index(ga_az, double, i);
    ad->accel_z[i] = *gaz;

    double* gmx = &g_array_index(ga_mx, double, i);
    ad->mag_x[i] = *gmx;
    double* gmy = &g_array_index(ga_my, double, i);
    ad->mag_y[i] = *gmy;
    double* gmz = &g_array_index(ga_mz, double, i);
    ad->mag_z[i] = *gmz;
  }

  return ad;
}

void ahrs_data_save_state_euler(struct ahrs_data* ad, int idx, double* X, double* P) {
  ad->est_phi[idx] = X[0];
  ad->est_theta[idx] = X[1];
  ad->est_psi[idx] = X[2];

  ad->est_bias_p[idx] = X[3];
  ad->est_bias_q[idx] = X[4];
  ad->est_bias_r[idx] = X[5];

  ad->P11[idx] = P[0];
  ad->P22[idx] = P[1*7 + 1];
  ad->P33[idx] = P[2*7 + 2];
  ad->P44[idx] = P[3*7 + 3];
  ad->P55[idx] = P[4*7 + 4];
  ad->P66[idx] = P[5*7 + 5];

}

void ahrs_data_save_state_quat(struct ahrs_data* ad, int idx, double* X, double* P) {
  double eulers[3];
  eulers_of_quat(eulers, X);
  ad->est_phi[idx] = eulers[0];
  ad->est_theta[idx] = eulers[1];
  ad->est_psi[idx] = eulers[2];

  ad->est_bias_p[idx] = X[4];
  ad->est_bias_q[idx] = X[5];
  ad->est_bias_r[idx] = X[6];

  ad->P11[idx] = P[0];
  ad->P22[idx] = P[1*7 + 1];
  ad->P33[idx] = P[2*7 + 2];
  ad->P44[idx] = P[3*7 + 3];
  ad->P55[idx] = P[4*7 + 4];
  ad->P66[idx] = P[5*7 + 5];
  ad->P77[idx] = P[6*7 + 6];

}

void ahrs_data_save_measure(struct ahrs_data* ad, int idx) {

  ad->m_phi[idx] = phi_of_accel(ad->accel_x[idx], ad->accel_y[idx], ad->accel_z[idx]);
  ad->m_theta[idx] = theta_of_accel(ad->accel_x[idx], ad->accel_y[idx], ad->accel_z[idx]);
  ad->m_psi[idx] = psi_of_mag(ad->est_phi[idx], ad->est_theta[idx],
			      ad->mag_x[idx], ad->mag_y[idx], ad->mag_z[idx]);

}
