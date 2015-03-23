#include "matrix.h"

#include <stdio.h>

void mat_add(int n_row, int n_col, double* r, double* a, double* b) {
  int row, col;
  for (row = 0; row<n_row; row++) {
    for (col = 0; col<n_col; col++) {
      int ridx = row * n_col + col;
      r[ridx] = a[ridx] + b[ridx];
    }
  }
}

void mat_sub(int n_row, int n_col, double* r, double* a, double* b) {
  int row, col;
  for (row = 0; row<n_row; row++) {
    for (col = 0; col<n_col; col++) {
      int ridx = row * n_col + col;
      r[ridx] = a[ridx] - b[ridx];
    }
  }
}

void mat_transpose(int n_row, int n_col, double* r, double* a) {
  int row, col;
  for (row = 0; row<n_row; row++) {
    for (col = 0; col<n_col; col++) {
      int aidx = row * n_col + col;
      int ridx = col * n_row + row;
      r[ridx] = a[aidx];
    }
  }
}

void mat_scal_mult(int n_row, int n_col, double* r, double k, double* a) {
  int row, col;
  for (row = 0; row<n_row; row++) {
    for (col = 0; col<n_col; col++) {
      int ridx = row * n_col + col;
      r[ridx] = k * a[ridx];
    }
  }
}

void mat_add_scal_mult(int n_row, int n_col, double* r, double*a, double k, double* b) {
  int row, col;
  for (row = 0; row<n_row; row++) {
    for (col = 0; col<n_col; col++) {
      int ridx = row * n_col + col;
      r[ridx] = a[ridx] + k * b[ridx];
    }
  }
}

void mat_mult(int n_rowa, int n_cola, int n_colb, double* r, double* a, double* b) {
  int row, col, k;
  for (row = 0; row<n_rowa; row++) {
    for (col = 0; col<n_colb; col++) {
      int ridx = col + row * n_colb;
      r[ridx] =0.;
      for (k=0; k<n_cola; k++) {
	int aidx = k + row * n_cola;
	int bidx = col + k * n_colb;
	r[ridx] += a[aidx] * b[bidx];
      }
    }
  }
}

void mat_print(int n_row, int n_col, double* a) {
  int row, col;
  for (row = 0; row<n_row; row++) {
    for (col = 0; col<n_col; col++) {
      int ridx = row * n_col + col;
      printf("%lf\t", a[ridx]);
    }
    printf("\n");
  }
}
