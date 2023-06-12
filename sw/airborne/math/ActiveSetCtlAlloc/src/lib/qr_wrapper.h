/**
 * Copyright (C) Till Blaha 2022-2023
 * MAVLab -- Faculty of Aerospace Engineering -- Delft University of Techology
 */

/**
 * @file qr_wrapper.h
 * 
 * @brief Defines wrappers to compute QR factors with qr_solve library
*/

#ifndef QR_WRAPPER_H
#define QR_WRAPPER_H

#include "solveActiveSet.h"

/** 
 * @brief Find unitary Q and upper triangular R, such that A = QR.
 * 
 * Uses qr_solve package from John Burkardt to do the heavy lifting.
 * http://people.sc.fsu.edu/~jburkardt/c_src/qr_solve/qr_solve.html
 *
 * Recovers explicit Q and R factors using dorgqr.
 * 
 * Allows permutation, such that A_p = QR is computed, where 
 * A_p[:, i] = A[:, perm[i]] for all i = 0 ... n
 * 
 * @param m number of rows in A
 * @param n number of columns in A
 * @param perm Permutation as decribed above
 * @param A pointer to the rows of matrix A
 * @param Q On exit: Holds pointers to rows of Q
 * @param R On exit: Holds pointers to rows of R (not space efficient)
 * 
 */
void qr_wrapper(int m, int n, int perm[], num_t** A, num_t** Q, num_t** R);

/**
 * @brief Recovers unitary Q from Householder factors of Q of QR factorisation
 * 
 * The matrix Q is represented as a product of elementary reflectors
 *
 *    Q = H(0) H(1) . . . H(k), where k = min(m,n)-1.
 *
 * Each H(i) has the form
 *
 *    H(i) = I - tau * v * v**T
 *
 * where tau is a real scalar, and v is a real vector with
 * v(0:i-1) = 0 and v(i) = 1; v(i+1:m-1) = A(i+1:m-1,i), and tau = TAU(i).
 * 
 * TODO: this is super slow. Much faster implementations are possible that
 * exploit the sparsity. See directly from Cleve Moler:
 * https://blogs.mathworks.com/cleve/2016/10/03/householder-reflections-and-the-qr-decomposition/
 * 
 * @param m number of rows in A
 * @param n number of columns in A
 * @param A Col-major representation of v-factor matrix. See above
 * @param Q On exit: Q factor of A = QR
 * @param tau Scalar factors in the Householder matrix. See above.
*/
int dorgqr ( int m, int n, const num_t A[], num_t Q[], num_t tau[]);

#endif