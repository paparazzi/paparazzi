#include<math.h>
#include<stdio.h>
#include"linalg.h"

void
ukf_cholesky_decomposition(double *A, unsigned n, double *sigma) {
	unsigned i,j,k;
	double t;

	for(i = 0 ; i < n ; i++) {
		if(i > 0) {
			for(j = i ; j < n ; j++) {
				t = 0.0;
				for(k = 0 ; k < i  ; k++)
					t += A[j * n + k] * A[i * n + k];
				A[j * n + i] -= t;
			}
		}
		if(A[i * n + i] <= 0.0) {
			fprintf(stderr, "Matrix is not positive definite \n");
			return;
		}
		sigma[i] = sqrt(A[i * n + i]);
		t = 1.0 / sigma[i];
		for(j = i ; j < n ; j++)
			A[j * n + i] *= t;
	}
}


void
ukf_cholesky_solve(double *A, unsigned n, double *sigma, double *B, unsigned m, double *X) {
	int i,j,k;
	double t;

	for(i = 0 ; i < m ; i++) { // iterate over the lines of B
		for(j = 0 ; j < n ; j++) { // solve Ly=B
			t = B[i * n + j];
			for(k = j - 1 ; k >= 0 ; k--)
				t -= A[j * n + k] * X[i * n + k];
			X[i * n + j] = t / sigma[j];
		}
		for(j = n - 1 ; j >= 0 ; j--) { // solve Ltx=y
			t = X[i * n + j];
			for(k = j + 1 ; k < n ; k++)
				t -= A[k * n + j] * X[i * n + k];
			X[i * n + j] = t / sigma[j];
		}
	}
}


void
ukf_cholesky_invert(double *A, unsigned n, double *sigma) {
	double t;
	int i,j,k;

	for(i = 0 ; i < n ; i++) {
		A[i * n + i] = 1.0 / sigma[i];
		for(j = i + 1 ; j < n ; j++) {
			t = 0.0;
			for(k = i ; k < j ; k++)
				t -= A[j * n + k] * A[k * n + i];
			A[j * n + i] = t / sigma[j];
		}
	}
}
