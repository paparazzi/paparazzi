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

  double p = u[0] - X[3];
  double q = u[1] - X[4];
  double r = u[2] - X[5];

  double phi   = X[0];
  double theta = X[1];
  //  double psi   = X[2];

  Xdot[0] = p + sin(phi)*tan(theta) * q + cos(phi)*tan(theta) * r;
  Xdot[1] =     cos(phi) * q            - sin(phi) * r;
  Xdot[2] =     sin(phi)/cos(theta) * q + cos(phi)/cos(theta) * r;
  Xdot[3] = 0.;
  Xdot[4] = 0.;
  Xdot[5] = 0.;

  F[0*6+0] =  cos(phi)*tan(theta)*q - sin(phi)*tan(theta)*r;
  F[0*6+1] =     1/(1+theta*theta) * (sin(phi)*q+cos(phi)*r);
  F[0*6+2] =  0.;
  F[0*6+3] =  -1.;
  F[0*6+4] =  -sin(phi)*tan(theta);
  F[0*6+5] =  -cos(phi)*tan(theta);

  F[1*6+0] = -sin(phi)*q - cos(phi)*r;
  F[1*6+1] = 0.;
  F[1*6+2] = 0.;
  F[1*6+3] = 0.;
  F[1*6+4] = -cos(phi);
  F[1*6+5] =  sin(phi);

  F[2*6+0] = cos(phi)/cos(theta)*q - sin(phi)/cos(theta)*r;
  F[2*6+1] = sin(theta)/(cos(theta)*cos(theta))*(sin(phi)*q+cos(phi)*r);
  F[2*6+2] = 0.;
  F[2*6+3] = 0.;
  F[2*6+4] = -sin(phi)/cos(theta);
  F[2*6+5] = -cos(phi)/cos(theta);

  F[3*6+0] = 0.;
  F[3*6+1] = 0.;
  F[3*6+2] = 0.;
  F[3*6+3] = 0.;
  F[3*6+4] = 0.;
  F[3*6+5] = 0.;

  F[4*6+0] = 0.;
  F[4*6+1] = 0.;
  F[4*6+2] = 0.;
  F[4*6+3] = 0.;
  F[4*6+4] = 0.;
  F[4*6+5] = 0.;

  F[5*6+0] = 0.;
  F[5*6+1] = 0.;
  F[5*6+2] = 0.;
  F[5*6+3] = 0.;
  F[5*6+4] = 0.;
  F[5*6+5] = 0.;

}

void linear_measure(double *y, double* err, double*X, double *H) {
  *err = y[0] - X[ahrs_state];
  switch (ahrs_state) {
  case UPDATE_PHI: {
    WARP(*err, M_PI);
    H[0] = 1.;
    H[1] = 0.;
    H[2] = 0.;
    break;
  }
  case UPDATE_THETA: {
    WARP(*err, M_PI_2);
    H[0] = 0.;
    H[1] = 1.;
    H[2] = 0.;
    break;
  }
  case UPDATE_PSI: {
    WARP(*err, M_PI);
    H[0] = 0.;
    H[1] = 0.;
    H[2] = 1.;
    break;
  }
  }
  H[3] = 0.;
  H[4] = 0.;
  H[5] = 0.;
}


#define P0E 1.0
#define P0B 1.0

#define QE 0.001
#define QB 0.008

void run_ekf (void) {
  /* initial state covariance matrix */
  double P[6*6] = {P0E,   0.0,   0.0,   0.0,   0.0,   0.0,
		   0.0,   P0E,   0.0,   0.0,   0.0,   0.0,
		   0.0,   0.0,   P0E,   0.0,   0.0,   0.0,
		   0.0,   0.0,   0.0,   P0B,   0.0,   0.0,
		   0.0,   0.0,   0.0,   0.0,   P0B,   0.0,
		   0.0,   0.0,   0.0,   0.0,   0.0,   P0B };
  /* model noise covariance matrix */
  double Q[6*6] = { QE,   0.0,   0.0,   0.0,   0.0,   0.0,
		   0.0,    QE,   0.0,   0.0,   0.0,   0.0,
		   0.0,   0.0,    QE,   0.0,   0.0,   0.0,
		   0.0,   0.0,   0.0,    QB,   0.0,   0.0,
		   0.0,   0.0,   0.0,   0.0,    QB,   0.0,
		   0.0,   0.0,   0.0,   0.0,   0.0,    QB };
  /* measurement noise covariance matrix */
  double R[1]={1.7};
  /* state [phi, theta, psi, bp, bq, br] */
  double X[6];
  /* measure */
  double Y[1];
  /* command */
  double U[3] = {0.0, 0.0, 0.0};

  struct ekf_filter* ekf;
  ekf = ekf_filter_new(6, 1, Q, R, linear_filter, linear_measure);
  ahrs_euler_init(ad, 150, X);
  ekf_filter_reset(ekf, X, P);
  int iter;

  for (iter=0; iter < ad->nb_samples; iter++) {
    U[0] = ad->gyro_p[iter];
    U[1] = ad->gyro_q[iter];
    U[2] = ad->gyro_r[iter];
    ekf_filter_predict(ekf, U);

    ahrs_state = iter%3;
    switch (ahrs_state) {
    case UPDATE_PHI:
      Y[0] = phi_of_accel(ad->accel_x[iter], ad->accel_y[iter], ad->accel_z[iter]);
      break;
    case UPDATE_THETA:
      Y[0] = theta_of_accel(ad->accel_x[iter], ad->accel_y[iter], ad->accel_z[iter]);
      break;
    case UPDATE_PSI: {
      double m_phi = phi_of_accel(ad->accel_x[iter], ad->accel_y[iter], ad->accel_z[iter]);
      double m_theta = theta_of_accel(ad->accel_x[iter], ad->accel_y[iter], ad->accel_z[iter]);
      Y[0] = psi_of_mag(m_phi, m_theta, ad->mag_x[iter], ad->mag_y[iter], ad->mag_z[iter]);
      break;
    }
    }
    ekf_filter_update(ekf, Y);

    ekf_filter_get_state(ekf, X, P);
    ahrs_data_save_state_euler(ad, iter, X, P);
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
