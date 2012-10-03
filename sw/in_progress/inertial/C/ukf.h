#ifndef UKF_H_
#define UKF_H_

/*
 * Opaque structure holding filter information
 */
typedef struct ukf_filter_t *ukf_filter;
/*
 * Basic functions describing evolution and measure
 */
typedef void (*filter_function)(double*, double *,double *);
typedef void (*measure_function)(double *, double *);

/*
 * creates a new unscented kalman filter structure
 * @param state_dim: size of state variable
 * @param mesaure_dim: size of measurement
 * @param Q: additive model noise covariance matrix
 * @param R: additive measurement noise covariance matrix
 * @param ffun: the funcion describing the filter evolution equation
 * @param mfun: the measurement function
 */
ukf_filter
ukf_filter_new(unsigned state_dim,
			   unsigned measure_dim,
			   double *Q,
			   double *R,
			   filter_function ffun,
			   measure_function mfun);

/*
 * free filter memory
 * @param filter: the filter to delete
 */
void
ukf_filter_delete(ukf_filter filter);

/*
 * set filter weight using default procedure
 * @param alpha: spread parameter
 * @param k: scaling parameter
 * @param beta: distribution fitting parameter (2 for Gaussian)
 */
 void
 ukf_filter_compute_weights(ukf_filter filter,
 					 double alpha,
 					 double k,
 					 double beta);

/*
 * Reset filter to new state
 * @param x0: initial state
 * @param P0: initial state covariance matrix
 */
void
ukf_filter_reset(ukf_filter filter,
				 double *x0,
				 double *PO);

/*
 * Get filter state
 * @param x: A vector that will hold the state
 * @param P: A vector that will hold the state covariance
 */
void
ukf_filter_get_state(ukf_filter filter, double *x, double *P);

/*
 * Update filter using a measure
 * @param y: The measure vector
 * @param u: the command
 */
void
ukf_filter_update(ukf_filter filter, double *y, double *u);

#endif /*UKF_H_*/
