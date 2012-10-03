#include "ekf.h"

#include "matrix.h"

#include <stdlib.h>
#include <string.h>
#include <stdio.h>

struct ekf_filter {
  unsigned state_dim;
  unsigned measure_dim;

  /* state                           */
  double* X;
  /* state covariance matrix         */
  double* P;
  /* process covariance noise        */
  double* Q;
  /* measurement covariance noise    */
  double* R;
  /* jacobian of Xdot wrt X          */
  double* F;
  /* jacobian of the measure wrt X   */
  double* H;
  /* error matrix                    */
  double* E;
  /* kalman gain                     */
  double* K;

  filter_function ffun;
  measure_function mfun;

  /* temps */
  double* Xdot;
  double* Pdot;
  double* tmp1;
  double* tmp2;
  double* tmp3;

};


struct ekf_filter* ekf_filter_new(unsigned state_dim,
				  unsigned measure_dim,
				  double* Q,
				  double* R,
				  filter_function ffun,
				  measure_function mfun) {

  struct ekf_filter* ekf = malloc(sizeof(struct ekf_filter));
  ekf->state_dim = state_dim;
  ekf->measure_dim = measure_dim;

  int n = ekf->state_dim;
  ekf->X = malloc( n * sizeof(double));
  ekf->Xdot = malloc( n * sizeof(double));

  n = ekf->state_dim * ekf->state_dim;
  ekf->P = malloc( n * sizeof(double));
  ekf->Pdot = malloc( n * sizeof(double));
  ekf->tmp1 = malloc( n * sizeof(double));
  ekf->tmp2 = malloc( n * sizeof(double));
  ekf->tmp3 = malloc( n * sizeof(double));

  ekf->Q = malloc( n * sizeof(double));
  memcpy(ekf->Q, Q, n * sizeof(double));

  n = ekf->measure_dim * ekf->measure_dim;
  ekf->R = malloc( n * sizeof(double));
  memcpy(ekf->R, R, n * sizeof(double));

  n = ekf->state_dim * ekf->state_dim;
  ekf->F = malloc( n * sizeof(double));

  n = ekf->measure_dim * ekf->state_dim;
  ekf->H = malloc( n * sizeof(double));

  n = ekf->measure_dim * ekf->measure_dim;
  ekf->E = malloc( n * sizeof(double));

  n = ekf->state_dim;
  ekf->K = malloc( n * sizeof(double));


  ekf->ffun = ffun;
  ekf->mfun = mfun;

  return ekf;
}


void ekf_filter_reset(struct ekf_filter *filter, double *x0, double *P0) {
  memcpy(filter->X, x0, filter->state_dim * sizeof(double));
  memcpy(filter->P, P0, filter->state_dim * filter->state_dim * sizeof(double));
}

void
ekf_filter_get_state(struct ekf_filter* filter, double *X, double* P){
  memcpy(X, filter->X, filter->state_dim * sizeof(double));
  memcpy(P, filter->P, filter->state_dim * filter->state_dim * sizeof(double));
}

void ekf_filter_predict(struct ekf_filter* filter, double *u) {

  /* prediction :

     X += Xdot * dt
     Pdot = F * P * F' + Q   ( or Pdot = F*P + P*F' + Q  for continuous form )
     P += Pdot * dt

  */

  int n = filter->state_dim;
  double dt;
  /* fetch dt, Xdot and F */
  filter->ffun(u, filter->X, &dt, filter->Xdot, filter->F);
  /*  X = X + Xdot * dt */
  mat_add_scal_mult(n, 1, filter->X, filter->X, dt, filter->Xdot);

#ifdef EKF_UPDATE_CONTINUOUS
  /*
      continuous update
      Pdot = F * P + P * F' + Q
  */
  mat_mult(n, n, n, filter->tmp1, filter->F, filter->P);
  mat_transpose(n, n, filter->tmp2, filter->F);
  mat_mult(n, n, n, filter->tmp3, filter->P, filter->tmp2);
  mat_add(n, n, filter->Pdot, filter->tmp1, filter->tmp3);
  mat_add(n, n, filter->Pdot, filter->Pdot, filter->Q);
#endif
#ifdef EKF_UPDATE_DISCRETE
  /*
      discrete update
      Pdot = F * P * F' + Q
  */
  mat_mult(n, n, n, filter->tmp1, filter->F, filter->P);
  mat_transpose(n, n, filter->tmp2, filter->F);
  mat_mult(n, n, n, filter->tmp3, filter->tmp1, filter->tmp2);
  mat_add(n, n, filter->Pdot, filter->tmp3, filter->Q);
#endif

  /*  P = P + Pdot * dt */
  mat_add_scal_mult(n, n, filter->P, filter->P, dt, filter->Pdot);

}

void ekf_filter_update(struct ekf_filter* filter, double *y) {

  /* update

     E = H * P * H' + R
     K = P * H' * inv(E)
     P = P - K * H * P
     X = X + K * err

  */
  double err;
  int n = filter->state_dim;
  int m = filter->measure_dim;
  filter->mfun(y, &err, filter->X, filter->H);

  /*  E = H * P * H' + R */
  mat_mult(m, n, n, filter->tmp1, filter->H, filter->P);
  mat_transpose(m, n, filter->tmp2, filter->H);
  mat_mult(m, n, m, filter->tmp3, filter->tmp1, filter->tmp2);
  mat_add(m, m, filter->E, filter->tmp3, filter->R);

  /*  K = P * H' * inv(E) */
  mat_transpose(m, n, filter->tmp1, filter->H);
  mat_mult(n, n, m, filter->tmp2, filter->P, filter->tmp1);
  if (filter->measure_dim != 1) { printf("only dim 1 measure implemented for now\n"); exit(-1);}
  mat_scal_mult(n, m, filter->K, 1./filter->E[0], filter->tmp2);

  /* P = P - K * H * P */
  mat_mult(n, m, n, filter->tmp1, filter->K, filter->H);
  mat_mult(n, n, n, filter->tmp2, filter->tmp1, filter->P);
  mat_sub(n, n, filter->P, filter->P, filter->tmp2);

  /*  X = X + err * K */
  mat_add_scal_mult(n, m, filter->X, filter->X, err, filter->K);

}
