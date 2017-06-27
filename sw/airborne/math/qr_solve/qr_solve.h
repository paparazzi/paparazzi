/*
 * This is part of the qr_solve library from John Burkardt.
 * http://people.sc.fsu.edu/~jburkardt/c_src/qr_solve/qr_solve.html
 *
 * It is slightly modified to make it compile on simple microprocessors,
 * and to remove all dynamic memory.
 *
 * This code is distributed under the GNU LGPL license.
 */

void daxpy ( int n, float da, float dx[], int incx, float dy[], int incy );
float ddot ( int n, float dx[], int incx, float dy[], int incy );
float dnrm2 ( int n, float x[], int incx );
void dqrank ( float a[], int lda, int m, int n, float tol, int *kr, 
  int jpvt[], float qraux[] );
void dqrdc ( float a[], int lda, int n, int p, float qraux[], int jpvt[], 
  float work[], int job );
int dqrls ( float a[], int lda, int m, int n, float tol, int *kr, float b[], 
  float x[], float rsd[], int jpvt[], float qraux[], int itask );
void dqrlss ( float a[], int lda, int m, int n, int kr, float b[], float x[], 
  float rsd[], int jpvt[], float qraux[] );
int dqrsl ( float a[], int lda, int n, int k, float qraux[], float y[], 
  float qy[], float qty[], float b[], float rsd[], float ab[], int job );
void drotg ( float *sa, float *sb, float *c, float *s );
void dscal ( int n, float sa, float x[], int incx );
void dswap ( int n, float x[], int incx, float y[], int incy );
void qr_solve ( int m, int n, float a[], float b[], float x[] );
