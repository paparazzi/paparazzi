#include "wls_alloc.h"
#include <stdio.h>

void main(int argc, char** argv) {
	/*
	float A[3][2] = {{2.0, 3.0}, {0, 1.0}, {1.0, 0.0}};
	int pos[2] = {1, 0};
	*/
	float A[3][2] = {{2.0, 3.0}, {1.0, 0.0}, {0.0, 1.0}};
	int pos[2] = {0, 1};
	int n = 3;
	int m = 2;
	float H[2][2];
	int s_dense = 1;

	float * A_ptr[3] = {A[0], A[1], A[2]};
	float * H_ptr[3] = {H[0], H[1]};

	float Ap[2][3];
	float* Ap_ptr[2] = {Ap[0], Ap[1]};
	for (int i=0; i<n; i++) {
		for(int j=0; j<m; j++) {
			Ap_ptr[j][i] = A_ptr[i][j];
		}
	}



	block_diag_self_mult(n,m,A_ptr,H_ptr,s_dense,pos);
	for (int i=0; i<m; i++) {
		for (int j=0; j<m; j++) {
			printf("%f, ", H_ptr[i][j]);
		}
		printf("\n");
	}


	block_diag_mult(m,n,m,Ap_ptr,A_ptr,H_ptr,s_dense,false,pos);
	for (int i=0; i<m; i++) {
		for (int j=0; j<m; j++) {
			printf("%f, ", H_ptr[i][j]);
		}
		printf("\n");
	}

	float Hl[3][3];
	float* Hl_ptr[3] = {Hl[0], Hl[1], Hl[2]};

	block_diag_mult(n,m,n,A_ptr,Ap_ptr,Hl_ptr,s_dense,true,pos);
	for (int i=0; i<n; i++) {
		for (int j=0; j<n; j++) {
			printf("%f, ", Hl_ptr[i][j]);
		}
		printf("\n");
	}

	float x[3] = {1., 2., 3.};
	float *B[3] = {x, x+1, x+2};
	float Y[2][1];
	float * Y_ptr[2] = {Y[0], Y[1]};

	block_diag_mult(m, n, 1, Ap_ptr, B, Y_ptr, s_dense, false, pos);
	for (int j=0; j<m; j++) {
		printf("%f\n", Y_ptr[j][0]);
	}
}