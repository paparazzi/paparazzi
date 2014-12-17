/*
 * Copyright (C) 2014 Gautier Hattenberger
 *
 * This file is part of paparazzi.
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */

/**
 * @file pprz_matrix_decomp_float.h
 * @brief Matrix decompositions in floating point.
 *
 */

#ifndef PPRZ_MATRIX_DECOMP_FLOAT_H
#define PPRZ_MATRIX_DECOMP_FLOAT_H

#ifdef __cplusplus
extern "C" {
#endif

#include "std.h"

/** Cholesky decomposition
 *
 * http://rosettacode.org/wiki/Cholesky_decomposition#C
 *
 * @param out pointer to the output array [n x n]
 * @param in pointer to the input array [n x n]
 * @param n dimension of the matrix
 */
void pprz_cholesky_float(float **out, float **in, int n);

/** QR decomposition
 *
 * using Householder method
 *
 * http://rosettacode.org/wiki/QR_decomposition#C
 *
 * @param Q square orthogonal matrix Q [m x m]
 * @param R upper triangular matrix R [m x n]
 * @param in pointer to the input array [m x n]
 * @param m number of rows of the input matrix
 * @param n number of columns of the input matrix
 */
void pprz_qr_float(float **Q, float **R, float **in, int m, int n);

/** SVD decomposition
 *
 * --------------------------------------------------------------------- *
 * Reference:  "Numerical Recipes By W.H. Press, B. P. Flannery,         *
 *              S.A. Teukolsky and W.T. Vetterling, Cambridge            *
 *              University Press, 1986" [BIBLI 08].                      *
 *                                                                       *
 *                               C++ Release 2.0 By J-P Moreau, Paris    *
 *                                         (www.jpmoreau.fr)             *
 * http://jean-pierre.moreau.pagesperso-orange.fr
 * --------------------------------------------------------------------- *
 *
 * Given a matrix a(m,n), this routine computes its singular value decomposition,
 * A = U · W · Vt. The matrix U replaces a on output. The diagonal matrix of singular
 * values W is output as a vector w(n). The matrix V (not the transpose Vt) is output
 * as v(n,n).
 *
 * @param a input matrix [m x n] and output matrix U [m x n]
 * @param w output diagonal vector of matrix W [n]
 * @param v output square matrix V [n x n]
 * @param m number of rows of input the matrix
 * @param n number of columns of the input matrix
 * @return 0 (false) if convergence failed, 1 (true) if decomposition succed
 */
int pprz_svd_float(float **a, float *w, float **v, int m, int n);

/** SVD based linear solver
 *
 * Solves A · X = B for a vector X,
 * where A is specified by the arrays u, w, v as returned by pprz_svd_float.
 * m and n are the dimensions of a.
 * b(m) is the input right-hand side.
 * x(n) is the output solution vector.
 * No input quantities are destroyed, so the routine may be called sequentially with different b's.
 *
 * @param x solution of the system ([n x l] matrix)
 * @param u U matrix from SVD decomposition
 * @param w diagonal of the W matrix from the SVD decomposition
 * @param v V matrrix from SVD decomposition
 * @param b right-hand side input matrix from system to solve (column vector [m x l])
 * @param m number of rows of the matrix A
 * @param n number of columns of the matrix A
 * @param l number of columns of the matrix B
 */
void pprz_svd_solve_float(float **x, float **u, float *w, float **v, float **b, int m, int n, int l);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* PPRZ_MATRIX_DECOMP_FLOAT_H */
