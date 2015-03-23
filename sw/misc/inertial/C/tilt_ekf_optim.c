#include <math.h>

#include "tilt_data.h"
#include "tilt_display.h"
#include "tilt_utils.h"

static struct tilt_data* td;


#define CONTINUOUS

void run_tilt(void) {
  /* state */
  double X[2];
  /* state covariance matrix */
  double P[4] = { 1., 0.,
		  0., 1. };
  /* model noise covariance matrix */
  double Q[4]={0.001, 0.0,
	       0.0, 0.003 };
  /* jacobian of the measure wrt X */
  const double H[2] = { 1., 0. };
  /* measurement covariance noise    */
  const double R = 0.5;

  tilt_init(td, 150, X);

  int iter;
  for (iter=0; iter<td->nb_samples; iter++) {

    double rate = td->gyro[iter] - X[1];
    /* X += Xdot * dt */
    X[0] += rate * td->dt;

#ifdef EKF_UPDATE_CONTINUOUS
    /* Pdot = F*P + P*F' + Q
     * F = { 1, 0,
     *       0, 0 };
     */
    double Pdot[4] = { Q[0] - P[0*2 + 1] - P[1*2 + 0], -P[1*2 + 1],
		      -P[1*2 + 1]                    ,  Q[1*2 + 1] };
#endif
#ifdef EKF_UPDATE_DISCRETE
    /* Pdot = F*P*F' + Q */
    double Pdot[4] = { P[1*2 + 1] +Q[0*2 + 0], 0.,
		       0.                    ,  Q[1*2 + 1] };
#endif
    /* P += Pdot * dt */
    P[0*2 + 0] += Pdot[0*2 + 0] * td->dt;
    P[0*2 + 1] += Pdot[0*2 + 1] * td->dt;
    P[1*2 + 0] += Pdot[1*2 + 0] * td->dt;
    P[1*2 + 1] += Pdot[1*2 + 1] * td->dt;

    /* E = H * P * H' + R */
    const double PHt_0 = P[0*2 + 0] * H[0]; // + P[0*2 + 1] * H[1]
    const double PHt_1 = P[1*2 + 0] * H[0]; // + P[1*2 + 1] * H[1]
    const double E = H[0] * PHt_0 + // H[1] * PHt1
                     R;

    /* K = P * H' * inv(E) */
    const double K_0 = PHt_0 / E;
    const double K_1 = PHt_1 / E;

    /* P = P - K * H * P */
    const double t_0 = PHt_0; /* H[0] * P[0][0] + H[1] * P[1][0] */
    const double t_1 = H[0] * P[0*2+ 1]; /* + H[1] * P[1][1]  = 0 */
    P[0*2 + 0] -= K_0 * t_0;
    P[0*2 + 1] -= K_0 * t_1;
    P[1*2 + 0] -= K_1 * t_0;
    P[1*2 + 1] -= K_1 * t_1;

    /* X += K * err */
    double err =  td->m_angle[iter] - X[0];
    X[0] += K_0 * err;
    X[1] += K_1 * err;

    tilt_data_save_state(td, iter, X, P);

  }
}


int main(int argc, char *argv[]) {
  gtk_init(&argc, &argv);

  //td = tilt_data_gen();
  //td = tilt_data_read_log("data/log_ahrs_still");
  //td = tilt_data_read_log("data/log_ahrs_roll");
  td = tilt_data_read_log("data/log_ahrs_bug");
  //td = tilt_data_read_log("data/log_ahrs_yaw_pitched");

  run_tilt();

  tilt_display(td);

  gtk_main();

  return 0;
}
