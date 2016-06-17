
#include <inttypes.h>
#include <math.h>

#include "ahrs_quat_fast_ekf.h"
#include "ahrs_data.h"
#include "ahrs_display.h"


#define UPDATE_PHI   0
#define UPDATE_THETA 1
#define UPDATE_PSI   2
#define UPDATE_NB    3

int main(int argc, char** argv) {
  gtk_init(&argc, &argv);

  struct ahrs_data* ad = ahrs_data_read_log("../data/log_ahrs_bug");
  //struct ahrs_data* ad = ahrs_data_read_log("../data/log_ahrs_roll");
  //struct ahrs_data* ad = ahrs_data_read_log("../data/log_ahrs_yaw_pitched");

  int idx;
  int len = 150;
  double gp=0., gq=0., gr=0., ax = 0., ay=0., az=0., mx=0., my=0., mz=0.;
  for (idx=0; idx<len; idx++) {
    gp+=ad->gyro_p[idx]; gq+=ad->gyro_q[idx]; gr+=ad->gyro_r[idx];
    ax+=ad->accel_x[idx]; ay+=ad->accel_y[idx]; az+=ad->accel_z[idx];
    mx+=ad->mag_x[idx]; my+=ad->mag_y[idx]; mz+=ad->mag_z[idx];
  }
  gp/=len; gq/=len; gr/=len;
  ax/=len; ay/=len; az/=len;
  mx/=len; my/=len; mz/=len;
  int16_t mag_init[3] = {mx, my, mz};
  FLOAT_T accel_init[3] = {ax, ay, az};
  FLOAT_T gyro_init[3] =  {gp, gq, gr};
  afe_init(mag_init, accel_init, gyro_init);

  int i;
  for (i=0; i<ad->nb_samples; i++) {
    FLOAT_T gyro[3] = {ad->gyro_p[i], ad->gyro_q[i], ad->gyro_r[i]};
    afe_predict(gyro);
    int update_stage = i%UPDATE_NB;
    switch(update_stage) {
    case UPDATE_PHI: {
      FLOAT_T accel[3] = {ad->accel_x[i], ad->accel_y[i], ad->accel_z[i]};
      afe_update_phi(accel);
      break;
    }
    case UPDATE_THETA: {
      FLOAT_T accel[3] = {ad->accel_x[i], ad->accel_y[i], ad->accel_z[i]};
      afe_update_theta(accel);
      break;
    }
    case UPDATE_PSI: {
      int16_t mag[3] = {ad->mag_x[i], ad->mag_y[i], ad->mag_z[i]};
      afe_update_psi(mag);
      break;
    }
    }
    /* save state for displaying */
    double X[] = { afe_q0, afe_q1, afe_q2, afe_q3, afe_bias_p, afe_bias_q, afe_bias_r };
    double P[] = { afe_P[0][0], afe_P[0][1], afe_P[0][2], afe_P[0][3], afe_P[0][4], afe_P[0][5], afe_P[0][6],
		   afe_P[1][0], afe_P[1][1], afe_P[1][2], afe_P[1][3], afe_P[1][4], afe_P[1][5], afe_P[1][6],
		   afe_P[2][0], afe_P[3][1], afe_P[2][2], afe_P[2][3], afe_P[2][4], afe_P[2][5], afe_P[2][6],
		   afe_P[3][0], afe_P[4][1], afe_P[3][2], afe_P[3][3], afe_P[3][4], afe_P[3][5], afe_P[3][6],
		   afe_P[4][0], afe_P[5][1], afe_P[4][2], afe_P[4][3], afe_P[4][4], afe_P[4][5], afe_P[4][6],
		   afe_P[5][0], afe_P[6][1], afe_P[5][2], afe_P[5][3], afe_P[5][4], afe_P[5][5], afe_P[5][6],
		   afe_P[6][0], afe_P[7][1], afe_P[6][2], afe_P[6][3], afe_P[6][4], afe_P[6][5], afe_P[6][6]};
    //    printf("P66 %f\n", afe_P[6][6]);
    ahrs_data_save_state(ad, i, X, P);
    /* save measures for displaying */
    ahrs_data_save_measure(ad, i);

  }

  ahrs_display(ad);

  gtk_main();
  return 0;
}
