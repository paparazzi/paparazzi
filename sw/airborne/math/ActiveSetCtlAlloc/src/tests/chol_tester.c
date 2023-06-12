#include "solveActiveSet.h"

#define N 5

void
main(int argc, char** argv) {
	num_t L[N][N] = {{1, 0, 0, 0, 0}, {1, 2, 0, 0, 0}, {3, 2, 1, 0, 0}, {2, 2, 4, 1, 0}, {4, 4, 5, 6, 7 }};
	num_t H[N][N] = {{1, 1, 3, 2, 4}, {1, 5, 7, 6, 12}, {3, 7, 14, 14, 25}, {2, 6, 14, 25, 42}, {4, 12, 25, 42, 142}};
	num_t * L_ptr[N];
	num_t * H_ptr[N];
	num_t inv_diag[N];
	for (int i=0; i<N; i++) {
		L_ptr[i] = L[i];
		H_ptr[i] = H[i];
		inv_diag[i] = 1.0 / L_ptr[i][i];
	}

	choldel(L_ptr, inv_diag, 4, N);

	for (int i=0; i<N; i++) {
		for (int j=0; j<N; j++)
			printf("%f, ", L_ptr[i][j]);
		printf("\n");
	}
	printf("\n");

	int permutation[N] = {0, 1, 2, 3, 4};
	
	choladd(L_ptr, 4, inv_diag, H_ptr, permutation, 4);

	for (int i=0; i<N; i++) {
		for (int j=0; j<N; j++)
			printf("%f, ", L_ptr[i][j]);
		printf("\n");
	}
	printf("\n");

}