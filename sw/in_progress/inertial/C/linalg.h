#ifndef LINALG_H_
#define LINALG_H_

/*
 * Computes cholesky decomposition of A
 * @param A: a n x n matrix. On exit, the lower triangle
 * of A is overwritten with the cholesky decomposition
 * @param n: the dimension of the square matrix A
 * @param sigma: the vector of diagonal elements of the cholesky
 * decomposition
 */

void
ukf_cholesky_decomposition(double *A, unsigned n, double *sigma);

/*
 * Solve the linear system AX^t=B^t given the cholesky decomposition of A
 * @param A: the cholesky decomposition of A, as given by the
 * routine ukf_cholesky_decomposition
 * @param n: the size of the problem
 * @param sigma: the vector of diagonal elements of A
 * @param B: the right hand side of the linear system. Size n x m
 * @param m: the number of simultaneou systems to solve
 * @param X: a matrix holding the result
 */

void
ukf_cholesky_solve(double *A, unsigned n, double *sigma, double *B, unsigned m, double *X);



/*
 * Lower triangle of the inverse of the cholesky decomposition.
 * On exit, the lower triangle of A is overwritten with the lower
 * triangle of the inverse.
 * @param A: the cholesky decomposition of A, as given by the
 * routine ukf_cholesky_decomposition
 * @param n: the size of the problem
 * @param sigma: the vector of diagonal elements of A
 */
void
ukf_cholesky_invert(double *A, unsigned n, double *sigma);

#endif /*LINALG_H_*/
