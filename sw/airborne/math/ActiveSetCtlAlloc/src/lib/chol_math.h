/**
 * Copyright (C) Till Blaha 2022-2023
 * MAVLab -- Faculty of Aerospace Engineering -- Delft University of Techology
 */

/**
 * @file chol_math.h
 * 
 * @brief Cholesky decomposition and update functions
*/

#ifndef CHOL_MATH_H
#define CHOL_MATH_H

#include "solveActiveSet.h"

#include <stdbool.h>

/** 
 * @brief Cholesky decomposition. Taken from paparazzi math library
 *
 * http://rosettacode.org/wiki/Cholesky_decomposition#C
 *
 * @param out pointer to the output array [n x n]
 * @param in pointer to the input array [n x n]
 * @param inv_diag On exit: the reciprocals of the diagonal elements of out 
 * (speeds up cholesky_solve calculations considerably by avoiding divisions)
 * @param n dimension of the matrix
 */
void pprz_cholesky_float(num_t **out, num_t **in, num_t* inv_diag, int n);

/**
 * @brief solve LL^T x = b for x, where L is lower triangular
 * 
 * Antoine Drouin, 2007, modified
 * 
 * @param L pointer to the lower cholesky factor [n x n]
 * (not space efficient, lots of 0s)
 * @param inv_diag The reciprocals of the diagonal elements of L
 * @param n dimension of the matrix
 * @param b Pointer to the elements of b
 * @param x On exit: holds solution to LL^T x = b
*/
void cholesky_solve(num_t **L, num_t* inv_diag, int n, num_t *b, num_t *x);

/**
 * @brief Partial cholesky factor rank1 update
 * 
 * Lp = L[i1:(i2-1), i1:(i2-1)]
 * Lp <- chol(Lp Lp^T + vv^T) where v = L[i1:(i2-1), i1-1]
 * 
 * TODO: attribution of the algo
 * 
 * @param L On entry: existing chol factor. On exit: Updated cholesky factor
 * @param i1 i1-1 defines the column to be used for the update vector
 * @param i2 i2-1 defines endpoint for the column
*/
void cholup(num_t** L, int i1, int i2);

/**
 * @brief Update cholesky factor when a row/column of LL^T is removed
 * 
 * TODO: attribution of the algo
 * 
 * @param L On entry: existing chol factor. On exit: Updated cholesky factor
 * @param inv_diag The reciprocals of the diagonal elements of L
 * @param f 0-based index of the row/col of LL^T that are to be deleted
 * @param n dimension of LL^T before deletion
*/
void choldel(num_t** L, num_t* inv_diag, int f, int n);

/**
 * @brief Update cholesky factor when a row/column of LL^T is appended
 * 
 * The row/col that is appended at the end of LL^T is H[perm[1:nf]][perm[f]]
 * 
 * TODO: attribution of the algo
 * 
 * @param L On entry: existing chol factor. On exit: Updated cholesky factor
 * Note: allocated space for L must be one higher than nf
 * @param nf Dimension of LL^T before appending
 * @param inv_diag The reciprocals of the diagonal elements of L
 * @param H Pointer to matrix holding the row/column to be appended to LL^T
 * @param perm Permutation of columns/rows of H. See solveActiveSet_chol.c
 * @param f 0-based index in perm, such that H[perm[1:nf]][perm[f]] is the 
 * column/row to be appended to LL^T
*/
void choladd(num_t** L, int nf, num_t* inv_diag, num_t** H, int* perm, int f);

#endif