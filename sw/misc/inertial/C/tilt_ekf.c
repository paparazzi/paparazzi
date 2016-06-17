#include <math.h>

#include "tilt_data.h"
#include "tilt_display.h"
#include "tilt_utils.h"
#include "ekf.h"

/*
  Simple 2 state filter for hybridizing gyrometer and accelerometer
  on one axis.
  The filter wil track the gyro bias.

  state : [ angle gyro_bias ]

*/

static struct tilt_data* td;
static int iter;

void linear_filter(double *u, double* x, double* dt, double *Xdot, double* F) {
  /* state prediction
     angle += angle_dot * dt
     bias  += 0
  */
  *dt = td->dt;
  Xdot[0] = (u[0] - x[1]);
  Xdot[1] = 0.;
  /* Jacobian of xdot wrt the state
      F = [ d(angle_dot)/d(angle)     d(angle_dot)/d(gyro_bias) ]
          [ d(gyro_bias_dot)/d(angle) d(gyro_bias_dot)/d(gyro_bias) ]
  */
  F[0] = 0.;
  F[1] = -1.;
  F[2] = 0.;
  F[3] = 0.;
}

void linear_measure(double *y, double* err, double*x, double *H) {
  /* err = measure - estimate */
  *err = y[0] - x[0];
  /* H is the jacobian of the measure wrt X   */
  H[0] = 1.;
  H[1] = 0.;
}


void run_ekf(struct tilt_data* td) {
  /* model noise covariance matrix       */
  double Q[4]={0.001, 0.0,
	       0.0,   0.003};
  /* measurement noise covariance matrix */
  double R[1]={1.7};
  /* initial x */
  double X0[2] = {0.0, 0.0};
  /* initial state covariance matrix */
  double P0[4] = {1.0, 0.0,
		  0.0, 1.0};
  /* measure */
  double y[1];
  /* command */
  double u[1];

  struct ekf_filter* filter;
  filter = ekf_filter_new(2, 1, Q, R, linear_filter, linear_measure);
  tilt_init(td, 150, X0);
  ekf_filter_reset(filter, X0, P0);

  /* filter run */
  for (iter=0; iter<td->nb_samples; iter++) {
    u[0] = td->gyro[iter];
    y[0] = td->m_angle[iter];
    ekf_filter_predict(filter, u);
    ekf_filter_update(filter, y);
    ekf_filter_get_state(filter, X0, P0);
    tilt_data_save_state(td, iter, X0, P0);
  }
}

int
main(int argc, char *argv[]) {
  gtk_init(&argc, &argv);

  //td = tilt_data_gen();
  //td = tilt_data_read_log("data/log_ahrs_still");
  td = tilt_data_read_log("../data/log_ahrs_roll");
  //td = tilt_data_read_log("data/log_ahrs_bug");
  //td = tilt_data_read_log("data/log_ahrs_yaw_pitched");
  run_ekf(td);
  tilt_display(td);

  gtk_main();

  return 0;
}
