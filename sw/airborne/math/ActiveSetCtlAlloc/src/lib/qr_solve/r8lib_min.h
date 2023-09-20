/*
 * This file is a modified subset of the R8lib from John Burkardt.
 * http://people.sc.fsu.edu/~jburkardt/c_src/r8lib/r8lib.html
 *
 * It is the minimal set of functions from r8lib needed to use qr_solve.
 *
 * This code is distributed under the GNU LGPL license.
 */

#include "solveActiveSet.h"


void r8mat_copy_new ( int m, int n, num_t a1[], num_t a2[] );
num_t r8_epsilon ( void );
num_t r8mat_amax ( int m, int n, num_t a[] );
num_t r8_sign ( num_t x );
num_t r8_max ( num_t x, num_t y );
num_t *r8mat_transpose_new ( int m, int n, num_t a[] );
num_t *r8mat_mm_new ( int n1, int n2, int n3, num_t a[], num_t b[] );
num_t *r8mat_cholesky_factor ( int n, num_t a[], int *flag );
num_t *r8mat_mv_new ( int m, int n, num_t a[], num_t x[] );
num_t *r8mat_cholesky_solve ( int n, num_t l[], num_t b[] );
num_t *r8mat_l_solve ( int n, num_t a[], num_t b[] );
num_t *r8mat_lt_solve ( int n, num_t a[], num_t b[] );
num_t *r8mat_mtv_new ( int m, int n, num_t a[], num_t x[] );
num_t r8vec_max ( int n, num_t r8vec[] );
int i4_min ( int i1, int i2 );
int i4_max ( int i1, int i2 );
