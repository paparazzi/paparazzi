/**
 * Copyright (C) Till Blaha 2022-2023
 * MAVLab -- Faculty of Aerospace Engineering -- Delft University of Techology
 */

/**
 * @file sparse_math.h
 * 
 * @brief Defines some matrix/vector functions for commonly used sparsity
*/

#ifndef SPARSE_MATH_H
#define SPARSE_MATH_H

#include "solveActiveSet.h"

#include <stdint.h>
#include <stdbool.h>

/**
 * @brief Efficiently solve Ax = b for A upper triangular and square
 * 
 * Attribution: Rosetta code? idk, not the hardest algo
 * 
 * @param n Dimension of square A
 * @param A Pointer to rows of A
 * @param b Equation rhs
 * @param x On exit: solution to Ax = b
*/
void backward_tri_solve(int n, num_t** A, const num_t* b, num_t* x);

/**
 * @brief Efficiently compute b = Ax for A upper tri (not necessarily square)
 * 
 * Attribution: Rosetta code? idk, not the hardest algo
 * 
 * @param n Rows in A
 * @param m Columns in A
 * @param A Pointer to rows of A
 * @param b On exit: A*x
 * @param x x in A*x
*/
void tri_mult(int n, int m, num_t** A, const num_t* x, num_t* b);

/**
 * @brief Efficiently compute A^T*A for if A has dense part and sparse part
 * 
 * A(1:s_dense, 1:m) is dense
 * A(s_dense+pos(:), pos(:)) is diagonal
 * 
 * @param n Rows in A
 * @param m Columns in A
 * @param A Pointer to rows of A
 * @param H On exit: A^T*A
 * @param s_dense Number of dense rows at the top of A
 * @param pos Positions of the non-zeros in the sparse part
*/
void block_diag_self_mult(int n, int m, num_t** A, num_t** H, int s_dense, int* pos);

/**
 * @brief Efficiently compute Y=A*B for if A has dense part and sparse part
 * 
 * Not documenting, because not used after all
*/
void block_diag_mult(int n, int m, int p, num_t** A, num_t** B,
    num_t** Y, int s_dense, bool vertical, int* pos);

/**
 * @brief Check if which elements of x are not between vectors xmin/xmax
 * 
 * elementwise check that returns:
 * output[perm[i]] = +1 for each element where x[perm[i]] >= xmax[perm[i]
 * output[perm[i]] = -1 for each element where x[perm[i]] <= xmin[perm[i]
 * output[perm[i]] = 0 otherwise
 * 
 * @param n Will perform checks for i = 0 ... n-1
 * @param tol Relative and absolute tolerance to apply to the limits
 * @param x input vector
 * @param xmin minimum vector
 * @param xmax maximum vector
 * @param output  Output vector
 * @param perm permutation as defined above 
*/
int check_limits_tol(int n, num_t tol, num_t* x, const num_t* xmin, const num_t* xmax, int8_t* output, int* perm);

#endif