/*
 * This file is a modified subset of the R8lib from John Burkardt.
 * http://people.sc.fsu.edu/~jburkardt/c_src/r8lib/r8lib.html
 *
 * It is the minimal set of functions from r8lib needed to use qr_solve.
 *
 * This code is distributed under the GNU LGPL license.
 */

void r8mat_copy_new ( int m, int n, float a1[], float a2[] );
float r8_epsilon ( void );
float r8mat_amax ( int m, int n, float a[] );
float r8_sign ( float x );
float r8_max ( float x, float y );
float *r8mat_transpose_new ( int m, int n, float a[] );
float *r8mat_mm_new ( int n1, int n2, int n3, float a[], float b[] );
float *r8mat_cholesky_factor ( int n, float a[], int *flag );
float *r8mat_mv_new ( int m, int n, float a[], float x[] );
float *r8mat_cholesky_solve ( int n, float l[], float b[] );
float *r8mat_l_solve ( int n, float a[], float b[] );
float *r8mat_lt_solve ( int n, float a[], float b[] );
float *r8mat_mtv_new ( int m, int n, float a[], float x[] );
float r8vec_max ( int n, float r8vec[] );
int i4_min ( int i1, int i2 );
int i4_max ( int i1, int i2 );
