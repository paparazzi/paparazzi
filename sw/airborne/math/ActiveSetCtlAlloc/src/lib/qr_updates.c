/**
 * Copyright (C) Till Blaha 2022-2023
 * MAVLab -- Faculty of Aerospace Engineering -- Delft University of Techology
 */

/**
 * @file qr_updates.c
 * 
 * @brief Implementations of qr_updates.h
*/

#include "qr_updates.h"
#include "sparse_math.h"
#include <math.h>
#include <stdbool.h>

void qr_shift ( int n, int p, num_t** Q_ptr, num_t** R_ptr, int i, int j ) {
  if (i == j) {
    return;
  }

  // apply initial shift
  int N;
  if (i > j) {
    N = i - j;
    for (int l=0; l<n; l++) {
      num_t tempf = R_ptr[l][j];
      for (int k=j; k<i; k++) {
        R_ptr[l][k] = R_ptr[l][k+1];
      }
      R_ptr[l][i] = tempf;
    }
  } else {
    N = j - i;
    for (int l=0; l<n; l++) {
      num_t tempf = R_ptr[l][j];
      for (int k=j-1; k>=i; k--) {
        R_ptr[l][k+1] = R_ptr[l][k];
      }
      R_ptr[l][i] = tempf;
    }
  }
  #ifdef test
  printf("R_c = [");
  for (int i = 0; i < n; i++) {
    for (int j = 0; j < p; j++) {
      printf("%f ", R_ptr[i][j]);
    }
    printf(";\n");
  }
  printf("];\n\n");
  #endif

  num_t G[4];
  int j1;
  int i1;
  for (int k=0; k < N; k++) {
    if (j>i) {
      j1 = j-k-1;
      i1 = i;
    } else {
      j1 = j+k;
      i1 = j+k;
    }
    givens(R_ptr[j1][i1], R_ptr[j1+1][i1], G);
    givens_left_apply(p, R_ptr, G, j1, j1+1);
    #ifdef test
    printf("R_c = [");
    for (int i = 0; i < n; i++) {
      for (int j = 0; j < p; j++) {
        printf("%f ", R_ptr[i][j]);
      }
      printf(";\n");
    }
    printf("];\n\n");
    #endif
    G[1] *= -1; //possibly not the most efficient way to do a transpose...
    G[2] *= -1;
    givens_right_apply(n, Q_ptr, G, j1, j1+1);
    #ifdef test
    printf("Q_c = [");
    for (int i = 0; i < n; i++) {
      for (int j = 0; j < n; j++) {
        printf("%f ", Q_ptr[i][j]);
      }
      printf(";\n");
    }
    printf("];\n\n");
    printf("a");
    #endif
  }
}

void givens(num_t a, num_t b, num_t G[4]) {
  num_t sigma = 1/hypotf(a, b);
  num_t c = sigma * a;
  num_t s = -sigma * b;
  G[0] = c;
  G[1] = s;
  G[2] = -s;
  G[3] = c;
}

void givens_left_apply(int p, num_t** A, num_t* G, int row1, int row2) {
  num_t temp;
  for (int i=0; i<p; i++) {
    temp = G[0]*A[row1][i] + G[2]*A[row2][i];
    A[row2][i] = G[1]*A[row1][i] + G[3]*A[row2][i];
    A[row1][i] = temp;
  }
}

void givens_right_apply(int n, num_t** A, num_t* G, int col1, int col2) {
  num_t temp;
  for (int i=0; i<n; i++) {
    temp = A[i][col1]*G[0] + A[i][col2]*G[1]; 
    A[i][col2] = A[i][col1]*G[2] + A[i][col2]*G[3];
    A[i][col1] = temp;
  }
}
