#include"ukf.h"
#include"linalg.h"
#include<math.h>
#include<string.h>
#include<stdlib.h>
#include<stdio.h>

struct ukf_filter_t {
	// filter state
	unsigned state_dim;
	unsigned measure_dim;
	double *x;
	// measure
	double *y;
	// matrice de covariance de l'état
	double *P;
	// matrice de covariance du bruit de modèle (additif)
	double *Q;
	// matrice de covariance du bruit de mesure (additif)
	double *R;
	// fonction d'évolution
	filter_function ffun;
	// fonction de mesure
	measure_function mfun;
	// all fields below are specific to UKF
	// weights for computing the mean
	double *wm;
	// weights for computing the covariance
	double *wc;
	// scaling parameter
	double gamma;
	// sigma points (first one is filter state)
	double *sigma_point;
	// preallocated temporaries
	double *sigma;
	double *sigma_y;
	double *PM;
	double *PM_save;
	double *xm;
	double *ym;
	double *khi;
	double *khi_y;
	double *Pyy;
	double *Pxy;
	double *dx;
	double *dy;
	double *gain;
	double *KL;
};

void
ukf_safe_free(void *p) {
	if(p != 0) free(p);
}

/*
 * ukf_filter_new
 */

ukf_filter
ukf_filter_new(unsigned state_dim,
			   unsigned measure_dim,
			   double *Q,
			   double *R,
			   filter_function ffun,
			   measure_function mfun) {
    ukf_filter filter;
    int n;
    unsigned err = 0;
    // nothing to do if no state or measurement !
    if(state_dim == 0 || measure_dim == 0)
    	return 0;
	// alloc new structure
	filter = malloc(sizeof(struct ukf_filter_t));
	// returns 0 if allocation fails
	if(filter == 0)
		return 0;
	// fills the structure
	filter->state_dim = state_dim;
	filter->measure_dim = measure_dim;
	filter->ffun = ffun;
	filter->mfun = mfun;

	filter->x = malloc(state_dim * sizeof(double));
	err |= (filter->x == 0);

	filter->y = malloc(measure_dim * sizeof(double));
	err |= (filter->y == 0);

	n = state_dim * state_dim;

	filter->P = malloc(n * sizeof(double));
	err |= (filter->P == 0);

	n = 2 * state_dim + 1;

	filter->wm = malloc(n * sizeof(double));
	err |= (filter->wm == 0);

	filter->wc = malloc(n * sizeof(double));
	err |= (filter->wc == 0);

	filter->sigma_point = malloc(n * state_dim * sizeof(double));
	err |= (filter->sigma_point == 0);

	n = filter->state_dim;

	filter->sigma = malloc(n * sizeof(double));
	err |= (filter->sigma == 0);

	filter->PM = malloc(n * n * sizeof(double));
	err |= (filter->PM == 0);

	filter->PM_save = malloc(n * n * sizeof(double));
	err |= (filter->PM == 0);

	filter->xm = malloc(n * sizeof(double));
	err |= (filter->xm == 0);

	filter->ym = malloc(filter->measure_dim * sizeof(double));
	err |= (filter->ym == 0);

	n = 2 * filter->state_dim + 1;
	filter->khi = malloc(n * filter->state_dim * sizeof(double));
	err |= (filter->khi == 0);

	filter->khi_y = malloc(n * filter->measure_dim * sizeof(double));
	err |= (filter->khi_y == 0);

	n = filter->measure_dim;
	filter->Pyy = malloc(n * n * sizeof(double));
	err |= (filter->Pyy == 0);

	filter->Pxy = malloc(n * filter->state_dim * sizeof(double));
	err |= (filter->Pxy == 0);


	filter->dx = malloc(filter->state_dim * sizeof(double));
	err |= (filter->dx == 0);

	filter->dy = malloc(filter->measure_dim * sizeof(double));
	err |= (filter->dy == 0);

	filter->gain = malloc(filter->state_dim * filter->measure_dim * sizeof(double));
	err |= (filter->gain == 0);

	filter->sigma_y = malloc(filter->measure_dim * sizeof(double));
	err |= (filter->sigma_y == 0);

	filter->KL = malloc(filter->state_dim * filter->measure_dim * sizeof(double));
	err |= (filter->KL == 0);

	if(err != 0) {
		ukf_filter_delete(filter);
		return 0;
	}

	n = filter->state_dim;
	filter->Q = malloc(n * n * sizeof(double));
	if(filter->Q == 0) {
		ukf_filter_delete(filter);
		return 0;
	}
	memcpy(filter->Q, Q, n * n * sizeof(double));

	n = filter->measure_dim;
	filter->R = malloc(n * n * sizeof(double));
	if(filter->R == 0) {
		ukf_filter_delete(filter);
		return 0;
	}
	memcpy(filter->R, R, n * n * sizeof(double));

	// returns the newly allocated structure
	return filter;
}

/*
 * ukf_filter_delete
 */

void
ukf_filter_delete(ukf_filter filter) {
	if(filter != 0) {
		ukf_safe_free(filter->x);
		ukf_safe_free(filter->y);
		ukf_safe_free(filter->P);
		ukf_safe_free(filter->Q);
		ukf_safe_free(filter->R);
		ukf_safe_free(filter->wm);
		ukf_safe_free(filter->wc);
		ukf_safe_free(filter->sigma_point);
		ukf_safe_free(filter->sigma);
		ukf_safe_free(filter->PM);
		ukf_safe_free(filter->PM_save);
		ukf_safe_free(filter->xm);
		ukf_safe_free(filter->ym);
		ukf_safe_free(filter->khi);
		ukf_safe_free(filter->khi_y);
		ukf_safe_free(filter->Pyy);
		ukf_safe_free(filter->Pxy);
		ukf_safe_free(filter->dx);
		ukf_safe_free(filter->dy);
		ukf_safe_free(filter->gain);
		ukf_safe_free(filter->sigma_y);
		ukf_safe_free(filter->KL);
		free(filter);
	}
}

/*
 * ukf_filter_compute_weights
 */

void
ukf_filter_compute_weights(ukf_filter filter,
 					 double alpha,
 					 double k,
 					 double beta) {
 	double l;
 	double lam;
 	unsigned i;

 	if(filter == 0) return;
 	l = (double)filter->state_dim;
 	// lambda parameter
 	lam = alpha * alpha *( l + k) - l;

 	filter->wm[0] = lam / (lam + l);
 	filter->wc[0] = filter->wm[0] + (1.0 - alpha * alpha + beta);
 	for(i = 1 ; i <= 2*filter->state_dim ; i++) {
 		filter->wm[i] = 0.5 / (lam + l);
 		filter->wc[i] = 0.5 / (lam + l);
 	}
 	filter->gamma = alpha*sqrt(l + k);
}


/*
 * ukf_filter_reset
 */

void
ukf_filter_reset(ukf_filter filter,
				 double *x0,
				 double *P0) {
	if(filter != 0) {
		// state of the filter
		memcpy(filter->x, x0, filter->state_dim * sizeof(double));
		memcpy(filter->P, P0,
			   filter->state_dim * filter->state_dim * sizeof(double));
	}
}

/*
 * ukf_filter_get_state
 */

void
ukf_filter_get_state(ukf_filter filter, double *x, double* P){
	if(filter != 0) {
		memcpy(x, filter->x, filter->state_dim * sizeof(double));
		memcpy(P, filter->P, filter->state_dim * filter->state_dim * sizeof(double));
	}
}


/*
 * ukf_filter_update
 */

void
ukf_filter_update(ukf_filter filter, double *y, double *u) {
	int l = filter->state_dim;
	int m = filter->measure_dim;
	int i,j,k;
	double t;

	// cholesky decomposition of the state covariance matrix

	ukf_cholesky_decomposition(filter->P, l, filter->sigma);

	//=================================
	// compute sigma points
	for(j = 0 ; j < l ; j++)
		filter->sigma_point[j]= filter->x[j];
	for(i = 0 ; i < l ; i++) {
		for(j = 0 ; j < i ; j++) {
			filter->sigma_point[(i + 1) * l + j] = filter->x[j] ;
			filter->sigma_point[(i + 1 + l) * l + j] = filter->x[j] ;
		}
		filter->sigma_point[(i + 1) * l + i] = filter->x[i] + filter->gamma * filter->sigma[i];
		filter->sigma_point[(i + 1 + l) * l + i] = filter->x[i] - filter->gamma * filter->sigma[i];
		for(j = i + 1 ; j < l ; j++) {
			filter->sigma_point[(i + 1) * l + j] = filter->x[j]  + filter->gamma * filter->P[j * l + i];
			filter->sigma_point[(i + 1 + l) * l + j] = filter->x[j] - filter->gamma * filter->P[j * l + i];
		}
	}


	//=================================
	// propagate sigma points
	for(i = 0 ; i < 2 * l + 1 ; i++) {
		filter->ffun(&(filter->khi[i * l]) , &(filter->sigma_point[i * l]) , u);
	}

	// compute state prediction xm
	for(i = 0 ; i < l ; i++) {
		filter->xm[i] = filter->wm[0] * filter->khi[i];
		for(j = 1 ; j < 2 * l + 1 ; j++) {
			filter->xm[i] += filter->wm[j] * filter->khi[j * l + i];
		}
	}

	//================================
	// time update

	// start with state covariance matrix
	for(i = 0 ; i < l * l ; i++)
		filter->PM[i] = filter->Q[i];

    // accumulate covariances
    for(i = 0 ; i < 2 * l + 1 ; i++) {
    	for(j = 0 ; j < l ; j++)
    		filter->dx[j]= filter->khi[i * l + j] - filter->xm[j];
    	for(j = 0 ; j < l ; j++) {
    		for(k = 0 ; k < l ; k++) {
    			filter->PM[j * l + k] += filter->wc[i] * filter->dx[j] * filter->dx[k];
    		}
    	}
    }

    // save PM matrix

    for(i = 0 ; i < l * l ; i++)
    	filter->PM_save[i] = filter->PM[i];

    // redraw sigma points

    ukf_cholesky_decomposition(filter->PM, l, filter->sigma);

	for(j = 0 ; j < l ; j++)
		filter->sigma_point[j]= filter->xm[j];
	for(i = 0 ; i < l ; i++) {
		for(j = 0 ; j < i ; j++) {
			filter->sigma_point[(i + 1) * l + j] = filter->xm[j] ;
			filter->sigma_point[(i + 1 + l) * l + j] = filter->xm[j] ;
		}
		filter->sigma_point[(i + 1) * l + i] = filter->xm[i] + filter->gamma * filter->sigma[i];
		filter->sigma_point[(i + 1 + l) * l + i] = filter->xm[i] - filter->gamma * filter->sigma[i];
		for(j = i + 1 ; j < l ; j++) {
			filter->sigma_point[(i + 1) * l + j] = filter->xm[j] + filter->gamma * filter->PM[j * l + i];
			filter->sigma_point[(i + 1 + l) * l + j] = filter->xm[j] - filter->gamma * filter->PM[j * l + i];
		}
	}



	//=================================
	// propagate measurement

	for(i = 0 ; i < 2 * l + 1 ; i++) {
		filter->mfun(&(filter->khi_y[i * m]) , &(filter->sigma_point[i * l]) );
	}

	// measurement prediction

	for(i = 0 ; i < m ; i++) {
		filter->ym[i] = filter->wm[0] * filter->khi_y[i];
		for(j = 1 ; j < 2 * l + 1 ; j++)
			filter->ym[i] += filter->wm[j] * filter->khi_y[j * m + i];
	}

	// measurement update

	// Pyy matrix
	// start with measure covariance matrix

	for(i = 0 ; i < m * m ; i++)
		filter->Pyy[i] = filter->R[i];

	// accumulate covariances

	for(i = 0 ; i < 2 * l + 1 ; i++) {
    	for(j = 0 ; j < m ; j++)
    		filter->dy[j]= filter->khi_y[i * m + j] - filter->ym[j];
    	for(j = 0 ; j < m ; j++)
    		for(k = 0 ; k < m ; k++) {
    			filter->Pyy[j * m + k] += filter->wc[i] * filter->dy[j] * filter->dy[k];
    		}
    }

     // Pxy matrix

    for(i = 0 ; i < m * l ; i++)
		filter->Pxy[i] = 0.0;

	// accumulate covariances

	for(i = 0 ; i < 2 * l + 1 ; i++) {
    	for(j = 0 ; j < m ; j++) {
    		filter->dy[j]= filter->khi_y[i * m + j] - filter->ym[j];
    	}
    	for(j = 0 ; j < l ; j++) {
    		filter->dx[j] = filter->sigma_point[i * l + j] - filter->xm[j];
    	}
    	for(j = 0 ; j < l ; j++)
    		for(k = 0 ; k < m ; k++) {
    			filter->Pxy[j * m + k] += filter->wc[i] * filter->dx[j] * filter->dy[k];
    		}
    }

    // gain de kalman

    ukf_cholesky_decomposition(filter->Pyy, m, filter->sigma_y);
    ukf_cholesky_solve(filter->Pyy, m, filter->sigma_y, filter->Pxy, l, filter->gain);

    // restore PM matrix

    for(i = 0 ; i < l * l ; i++)
    	filter->P[i]= filter->PM_save[i];

    // update state

    for(j = 0 ; j < m ; j++)
    	filter->dy[j] = y[j] - filter->ym[j];
    for(i = 0 ; i < l ; i++) {
    	filter->x[i] = filter->xm[i];
    	t = 0.0;
    	for(j = 0 ; j < m ; j++)
    		t += filter->gain[i * m + j] * filter->dy[j];
    	filter->x[i] += t;
    }

    for(i = 0 ; i < l ; i++)
    	for(j = 0 ; j < m ; j++) {
    		t = 0.0;
    		for(k = j ; k < m ; k++)
    			t += filter->gain[i * m + k] * filter->Pyy[k * m + j];
    		filter->KL[i * m + j] = t;
    	}
    for(i = 0 ; i < l ; i++)
    	for(j = 0 ; j < l ; j++) {
    		t = 0.0;
    		for(k = 0 ; k < m ; k++)
    			t += filter->KL[i * m + k] * filter->KL[j * m + k];
    		filter->P[i * l + j ] -= t;
    	}

   // finished with kalman iteration !

}

