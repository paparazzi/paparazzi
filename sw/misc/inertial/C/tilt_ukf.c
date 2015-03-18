#include <math.h>

#include "tilt_data.h"
#include "tilt_display.h"
#include "tilt_utils.h"
#include "ukf.h"

/*
  Simple 2 state filter for hybridizing gyrometer and accelerometer
  on one axis.
  The filter wil track the gyro bias.

  state : [ angle gyro_bias ]

*/
static struct tilt_data* td;

/* x1 = A.x0 + B + u */
void linear_filter(double *x1, double *x0, double *u) {
  //  printf("in linear_filter (%d)\n", iter);
  x1[0] = x0[0] + (u[0] - x0[1])*td->dt;
  x1[1] = x0[1];

}

/* y = H.x + C */
void linear_measure(double *y, double *x) {
  //  printf("in linear_measure (%d)\n", iter);
  y[0] = x[0];
}


void run_ukf(struct tilt_data* td) {
  /* model noise covariance matrix */
  double Q[4]={0.001, 0.0,
	       0.0, 0.003 };
  /* measurement noise covariance matrix */
  double R[1]={0.5};
  /* initial x */
  double x[2] = {0.0, 0.0};
  /* initial state covariance matrix */
  double P[4] = {1.0, 0.0,
		 0.0, 1.0};
  /* measure */
  double y[1];
  /* command */
  double u[2] = {0.0, 0.0};

  ukf_filter filter;
  filter = ukf_filter_new(2, 1, Q, R, linear_filter, linear_measure);
  tilt_init(td, 150, x);
  ukf_filter_reset(filter, x, P);
  ukf_filter_compute_weights(filter, 1.1, 0.0, 2.0);

  /* filter run */
  int iter;
  for (iter=0; iter<td->nb_samples; iter++) {
    u[0] = td->gyro[iter];
    y[0] = td->m_angle[iter];
    ukf_filter_update(filter, y, u);
    ukf_filter_get_state(filter, x, P);
    tilt_data_save_state(td, iter, x, P);
  }
  ukf_filter_delete(filter);
}

int
main(int argc, char *argv[]) {
  gtk_init(&argc, &argv);

  //td = tilt_data_gen();
  //td = tilt_data_read_log("../data/log_ahrs_still");
  td = tilt_data_read_log("../data/log_ahrs_roll");
  //td = tilt_data_read_log("../data/log_ahrs_bug");
  //td = tilt_data_read_log("../data/log_ahrs_yaw_pitched");
  run_ukf(td);
  tilt_display(td);

  gtk_main();

  return 0;
}
