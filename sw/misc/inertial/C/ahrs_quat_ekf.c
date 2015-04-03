#include "ahrs_data.h"
#include "ahrs_display.h"
#include "ahrs_utils.h"
#include "ekf.h"

#include <string.h>
#include <math.h>

static struct ahrs_data* ad;
#define UPDATE_PHI   0
#define UPDATE_THETA 1
#define UPDATE_PSI   2
#define UPDATE_NB    3
static int ahrs_state;

void linear_filter(double *u, double* X, double* dt, double *Xdot, double* F) {
  *dt = ad->dt;

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
  double p = u[0] - X[4];
  double q = u[1] - X[5];
  double r = u[2] - X[6];
  double q0 = X[0];
  double q1 = X[1];
  double q2 = X[2];
  double q3 = X[3];

  double q0_dot =      -p*q1 -q*q2 -r*q3;
  double q1_dot = p*q0       +r*q2 -q*q3;
  double q2_dot = q*q0 -r*q1       +p*q3;
  double q3_dot = r*q0 +q*q1 -p*q2      ;

  Xdot[0] = q0_dot;
  Xdot[1] = q1_dot;
  Xdot[2] = q2_dot;
  Xdot[3] = q3_dot;
  Xdot[4] = 0.;
  Xdot[5] = 0.;
  Xdot[6] = 0.;

  /*
   * F is the Jacobian of Xdot wrt the state
   *
   *
   *
   */
  double my_f[] = { 0., -p, -q, -r,  q1,  q2,  q3,
		    p , 0.,  r, -q, -q0,  q3, -q2,
		    q,  -r, 0.,  p, -q3, -q0,  q1,
		    r,   q, -p, 0.,  q2, -q1, -q0,
		    0., 0., 0., 0.,  0.,  0.,  0.,
		    0., 0., 0., 0.,  0.,  0.,  0.,
		    0., 0., 0., 0.,  0.,  0.,  0. };
  memcpy(F, my_f, sizeof(my_f));

}

void linear_measure(double *y, double* err, double*X, double *H) {
  double eulers[3];
  eulers_of_quat(eulers, X);
  *err = y[0] - eulers[ahrs_state];
  /* H is the jacobian of the measure wrt X   */
  double q0 = X[0];
  double q1 = X[1];
  double q2 = X[2];
  double q3 = X[3];
  double dcm00 = 1.0-2*(q2*q2 + q3*q3);
  double dcm01 =     2*(q1*q2 + q0*q3);
  double dcm02 =     2*(q1*q3 - q0*q2);
  double dcm12 =     2*(q2*q3 + q0*q1);
  double dcm22 = 1.0-2*(q1*q1 + q2*q2);
  switch (ahrs_state) {
  case UPDATE_PHI: {
    WARP(*err, M_PI);
    const double phi_err = 2. / (dcm22*dcm22 + dcm12*dcm12);
    H[0] = (q1 * dcm22) * phi_err;
    H[1] = (q0 * dcm22 + 2 * q1 * dcm12) * phi_err;
    H[2] = (q3 * dcm22 + 2 * q2 * dcm12) * phi_err;
    H[3] = (q2 * dcm22) * phi_err;
    break;
  }
  case UPDATE_THETA: {
    WARP(*err, M_PI_2);
    const double theta_err = -2. / sqrt( 1 - dcm02*dcm02 );
    H[0] = -q2 * theta_err;
    H[1] =  q3 * theta_err;
    H[2] = -q0 * theta_err;
    H[3] =  q1 * theta_err;
    break;
  }
  case UPDATE_PSI: {
    WARP(*err, M_PI);
    const double psi_err = 2. / (dcm00*dcm00 + dcm01*dcm01);
    H[0] = (q3 * dcm00) * psi_err;
    H[1] = (q2 * dcm00) * psi_err;
    H[2] = (q1 * dcm00 + 2 * q2 * dcm01) * psi_err;
    H[3] = (q0 * dcm00 + 2 * q3 * dcm01) * psi_err;
    break;
  }
  }
  H[4] = 0.;
  H[5] = 0.;
  H[6] = 0.;
}

#define P0Q 1.0
#define P0B 1.0

#define Q0G 0.001
#define Q0B 0.008

void run_ekf (void) {
  /* initial state covariance matrix */
  double P[7*7] = {P0Q,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,
		   0.0,   P0Q,   0.0,   0.0,   0.0,   0.0,   0.0,
		   0.0,   0.0,   P0Q,   0.0,   0.0,   0.0,   0.0,
		   0.0,   0.0,   0.0,   P0Q,   0.0,   0.0,   0.0,
		   0.0,   0.0,   0.0,   0.0,   P0B,   0.0,   0.0,
		   0.0,   0.0,   0.0,   0.0,   0.0,   P0B,   0.0,
		   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   P0B };
  /* model noise covariance matrix */
  double Q[7*7] = {Q0G,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,
		   0.0,   Q0G,   0.0,   0.0,   0.0,   0.0,   0.0,
		   0.0,   0.0,   Q0G,   0.0,   0.0,   0.0,   0.0,
		   0.0,   0.0,   0.0,   Q0G,   0.0,   0.0,   0.0,
		   0.0,   0.0,   0.0,   0.0,   Q0B,   0.0,   0.0,
		   0.0,   0.0,   0.0,   0.0,   0.0,   Q0B,   0.0,
		   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   Q0B };
  /* measurement noise covariance matrix */
  double R[1]={1.7};
  /* state [q0, q1, q2, q3, bp, bq, br] */
  double X[7];
  /* measure */
  double Y[1];
  /* command */
  double U[3] = {0.0, 0.0, 0.0};

  struct ekf_filter* ekf;
  ekf = ekf_filter_new(7, 1, Q, R, linear_filter, linear_measure);
  ahrs_quat_init(ad, 150, X);
  ekf_filter_reset(ekf, X, P);

  /* filter run */
  int iter;
  for (iter=0; iter < ad->nb_samples; iter++) {
    U[0] = ad->gyro_p[iter];// - X[4];
    U[1] = ad->gyro_q[iter];// - X[5];
    U[2] = ad->gyro_r[iter];// - X[6];
    ekf_filter_predict(ekf, U);
    norm_quat(X);
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
    ekf_filter_update(ekf, Y);
    norm_quat(X);
    //    printf("P66 %f\n", P[6*7 + 6]);
    ekf_filter_get_state(ekf, X, P);
    ahrs_data_save_state_quat(ad, iter, X, P);
    ahrs_data_save_measure(ad, iter);
  }
}

int
main(int argc, char *argv[]) {
  gtk_init(&argc, &argv);

  ad = ahrs_data_read_log("../data/log_ahrs_bug");
  //ad = ahrs_data_read_log("../data/log_ahrs_roll");
  //ad = ahrs_data_read_log("../data/log_ahrs_yaw_pitched");
  run_ekf();
  ahrs_display(ad);
  gtk_main();

  return 0;
}
