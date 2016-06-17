#ifndef EKF_H
#define EKF_H

struct ekf_filter;

/*
 * Basic functions describing evolution and measure
 */
typedef void (*filter_function)(double*, double*, double *,double *, double*);
typedef void (*measure_function)(double *, double *, double*, double*);

extern struct ekf_filter* ekf_filter_new(unsigned state_dim,
					 unsigned measure_dim,
					 double* Q,
					 double* R,
					 filter_function ffun,
					 measure_function mfun);
extern void ekf_filter_reset(struct ekf_filter *filter, double *x0, double *P0);

extern void ekf_filter_predict(struct ekf_filter *filter, double *u);
extern void ekf_filter_update(struct ekf_filter *filter, double *y);

extern void ekf_filter_get_state(struct ekf_filter* filter, double *X, double* P);
#endif /* EKF_H */
