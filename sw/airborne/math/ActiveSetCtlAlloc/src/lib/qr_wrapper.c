/**
 * Copyright (C) Till Blaha 2022-2023
 * MAVLab -- Faculty of Aerospace Engineering -- Delft University of Techology
 */

/**
 * @file qr_wrapper.c
 * 
 * @brief Implementations of qr_wrapper.h
*/

#include "qr_wrapper.h"
#include "qr_solve/qr_solve.h"
#include <math.h>
#include <string.h>


void qr_wrapper(int m, int n, int perm[], num_t** A, num_t** Q, num_t** R) {
  num_t in[AS_N_C*AS_N_U]; // changed from VLA...
  int k = 0;
  for (int j = 0; j < n; j++) {
    for (int i = 0; i < m; i++) {
      in[k++] = A[i][perm[j]];
    }
  }
  int jpvt[AS_N_U];
  //int kr;
  num_t tau[AS_N_U];
  num_t work[AS_N_U];
  int job = 0;

  dqrdc ( in, m, m, n, tau, jpvt, work, job );
  num_t Qout[AS_N_C*AS_N_C];

  dorgqr ( m, n, in, Qout, tau);

  #ifdef debug_qr
  printf("in = [");
  for (int i = 0; i < m; i++) {
    for (int j = 0; j < n; j++) {
      printf("%f ", in[i + j*m]);
    }
    printf(";\n");
  }
  printf("];\n\n");

  printf("tau = [");
  for (int i = 0; i < m; i++) {
    printf("%f ", tau[i]);
    printf(";\n");
  }
  printf("];\n\n");
  #endif

  for (int i = 0; i < m*n; i++) {
    if (i%m > i/m) {
      R[i%m][i/m] = 0;
    } else {
      R[i%m][i/m] = in[i];
    }
  }

  for (int i = 0; i < m*m; i++)
    Q[i%m][i/m] = Qout[i];

}

int dorgqr ( int m, int n, const num_t a[], num_t q[], num_t tau[])
{
  // initialize to 0
  // TODO: is this necessary?
  for (int i = 0; i < m*m; i++) {
    q[i] = 0.;
  }
  for (int k=0; k<m; k++) {
    // start with identity
    q[k + k*m] = 1.0F;
  }
  num_t sqtau, isqtau, tsum;
  for (int i=0; i<n; i++) {
    sqtau = sqrtf(tau[i]);
    isqtau = 1/sqtau;

    for (int k=0; k<m; k++) {
      // tsum = Q(k, :)*v
      tsum = q[k + i*m] * sqtau; // *v(i), but v(i)=1
      for (int l=i+1; l<m; l++) {
        tsum += q[k + l*m] * a[l + i*m] * isqtau;
      }

      // Q(k, j) = Q(k, j) - tau(i)*(Q(k, :)*v)*v(j)
      q[k + i*m] -= tsum * sqtau; // *v(j), but v(j==i)=1
      for (int j=i+1; j<m; j++) {
        q[k + j*m] -= tsum*a[j + i*m] * isqtau;
      }
    }
  }
  // i'll eat my hat if this works first try
  return 0;
}
