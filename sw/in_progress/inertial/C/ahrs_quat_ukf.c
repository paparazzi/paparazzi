#include "ahrs_data.h"
#include "ahrs_display.h"
#include "ahrs_utils.h"
#include "ukf.h"

#define UPDATE_PHI   0
#define UPDATE_THETA 1
#define UPDATE_PSI   2
#define UPDATE_NB    3

static struct ahrs_data* ad;
static int ahrs_state;

void linear_filter(double *x1, double *x0, double *u) {
  /*
   * quat_dot = Wxq(pqr) * quat
   * bias_dot = 0
   *
   * Wxq is the quaternion omega matrix:
   *
   *              [ 0, -p, -q, -r ]
   *      1/2 *   [ p,  0,  r, -q ]
   *              [ q, -r,  0,  p ]
   *              [ r,  q, -p,  0 ]
   */
  double p = u[0] - x0[4];
  double q = u[1] - x0[5];
  double r = u[2] - x0[6];
  double q0_dot =         -p*x0[1] -q*x0[2] -r*x0[3];
  double q1_dot = p*x0[0]          +r*x0[2] -q*x0[3];
  double q2_dot = q*x0[0] -r*x0[1]          +p*x0[3];
  double q3_dot = r*x0[0] +q*x0[1] -p*x0[2];

  x1[0] = x0[0] + q0_dot * ad->dt;
  x1[1] = x0[1] + q1_dot * ad->dt;
  x1[2] = x0[2] + q2_dot * ad->dt;
  x1[3] = x0[3] + q3_dot * ad->dt;
  x1[4] = x0[4];
  x1[5] = x0[5];
  x1[6] = x0[6];

  norm_quat(x1);
}

void linear_measure(double *y, double *x) {
  double eulers[3];
  eulers_of_quat(eulers, x);
  y[0] = eulers[ahrs_state];
}



void run_ukf(void) {
  /* initial state covariance matrix */
  double P[7*7] = {1.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,
		   0.0,   1.0,   0.0,   0.0,   0.0,   0.0,   0.0,
		   0.0,   0.0,   1.0,   0.0,   0.0,   0.0,   0.0,
		   0.0,   0.0,   0.0,   1.0,   0.0,   0.0,   0.0,
		   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,
		   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,
		   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0 };
  /* model noise covariance matrix */
  double Q[7*7] = {0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,
		   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,
		   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,
		   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,
		   0.0,   0.0,   0.0,   0.0,   0.008, 0.0,   0.0,
		   0.0,   0.0,   0.0,   0.0,   0.0,   0.008, 0.0,
		   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.008 };
  /* measurement noise covariance matrix */
  double R[1]={0.3};
  /* state [q0, q1, q2, q3, bp, bq, br] */
  double X[7];
  /* measure */
  double Y[1];
  /* command */
  double U[3] = {0.0, 0.0, 0.0};

  ukf_filter filter;
  filter = ukf_filter_new(7, 1, Q, R, linear_filter, linear_measure);
  ahrs_init(ad, 150, X);
  ukf_filter_reset(filter, X, P);
  ukf_filter_compute_weights(filter, 1.1, 0.0, 2.0);

  int iter;
  for (iter=0; iter < ad->nb_samples; iter++) {
    U[0] = ad->gyro_p[iter];// - X[4];
    U[1] = ad->gyro_q[iter];// - X[5];
    U[2] = ad->gyro_r[iter];// - X[6];
    ahrs_state = iter%3;
    switch (ahrs_state) {
    case UPDATE_PHI:
      Y[0] = phi_of_accel(ad->accel_x[iter], ad->accel_y[iter], ad->accel_z[iter]);
      break;
    case UPDATE_THETA:
      Y[0] = theta_of_accel(ad->accel_x[iter], ad->accel_y[iter], ad->accel_z[iter]);
      break;
    case UPDATE_PSI: {
      double eulers[3];
      eulers_of_quat(eulers, X);
      Y[0] = psi_of_mag(eulers[0], eulers[1], ad->mag_x[iter], ad->mag_y[iter], ad->mag_z[iter]);
      break;
    }
    }
    ukf_filter_update(filter, Y, U);
    ukf_filter_get_state(filter, X, P);
    ahrs_data_save_state(ad, iter, X, P);
  }

  ukf_filter_delete(filter);
}

int main(int argc, char** argv) {
  gtk_init(&argc, &argv);

  //ad = ahrs_data_read_log("../data/log_ahrs_bug");
  ad = ahrs_data_read_log("../data/log_ahrs_roll");
  // ad = ahrs_data_read_log("../data/log_ahrs_yaw_pitched");
  run_ukf();
  ahrs_display(ad);
  gtk_main();
  return 0;
}
