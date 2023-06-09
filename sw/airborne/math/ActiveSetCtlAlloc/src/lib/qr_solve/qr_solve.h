/*
 * This is part of the qr_solve library from John Burkardt.
 * http://people.sc.fsu.edu/~jburkardt/c_src/qr_solve/qr_solve.html
 *
 * It is slightly modified to make it compile on simple microprocessors,
 * and to remove all dynamic memory.
 *
 * This code is distributed under the GNU LGPL license.
 */

#ifndef QR_SOLVE_H
#define QR_SOLVE_H

#include "solveActiveSet.h"


void daxpy ( int n, num_t da, num_t dx[], int incx, num_t dy[], int incy );
num_t ddot ( int n, num_t dx[], int incx, num_t dy[], int incy );
num_t dnrm2 ( int n, num_t x[], int incx );
void dqrank ( num_t a[], int lda, int m, int n, num_t tol, int *kr, 
  int jpvt[], num_t qraux[] );
void dqrdc ( num_t a[], int lda, int n, int p, num_t qraux[], int jpvt[], 
  num_t work[], int job );
int dqrls ( num_t a[], int lda, int m, int n, num_t tol, int *kr, num_t b[], 
  num_t x[], num_t rsd[], int jpvt[], num_t qraux[], int itask );
void dqrlss ( num_t a[], int lda, int m, int n, int kr, num_t b[], num_t x[], 
  num_t rsd[], int jpvt[], num_t qraux[] );
int dqrsl ( num_t a[], int lda, int n, int k, num_t qraux[], num_t y[], 
  num_t qy[], num_t qty[], num_t b[], num_t rsd[], num_t ab[], int job );
void drotg ( num_t *sa, num_t *sb, num_t *c, num_t *s );
void dscal ( int n, num_t sa, num_t x[], int incx );
void dswap ( int n, num_t x[], int incx, num_t y[], int incy );
void qr_solve ( int m, int n, num_t a[], num_t b[], num_t x[] );

#endif