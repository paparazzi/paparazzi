/**
 * Copyright (C) Till Blaha 2022-2023
 * MAVLab -- Faculty of Aerospace Engineering -- Delft University of Techology
 */

/**
 * @file setupWLS.h
 * 
 * @brief Defines conversion from weighted least-squares to least-squares
 */

#ifndef SETUP_WLS_H
#define SETUP_WLS_H

#include <solveActiveSet.h>

/**
 * @brief Converts WLS problem to LS with condition number limiting
 * 
 * Control alloc problem:
 *  min_u  ||Bu - v_des||_Wv^2  +  gamma^2*||u - ud||_Wu^2
 * 
 * LS problem:
 *  min_u  || Au - b ||_2
 * 
 * Outputs only A and gamma. b has to be computed with setupWLS_b
 * 
 * For the automatic scaling of the objectives, this uses the parameter theta,
 * which sets the target ratio of the largest eigenvalue of the first term and 
 * the largest eigenvalue of the second term.
 * If v_des and u are on similar scales, this approximates some separation of
 * the two objectives. It would be much more accurate to consider the lowest 
 * eigenvalue of the first term, but that is much harder to estimate quickly.
 * 
 * If v_des is generally larger than u (on a larger scale), then theta may be
 * chosen slightly lower.
 * 
 * Control allocation theta guidelines:
 * 1e-3
 * 
 * Control allocation cond_target guidelines:
 * 1e9 for QR and QR_NAIVE algorithms
 * 1e6 for Cholesky, and expect slight mixing of the objectives
 * 
 * Too high gamma (caused by too high theta or too low cond_bound) results in 
 * higher penalty on u (if these represent control actuators, this means a 
 * slower response).
 * Too low gamma (when cond_bound is too high _and_ theta is low), results in 
 * numerical inaccuracies, especially in the Cholesky algorithm, but should not
 * cause crashes.
 * 
 * @param B Effectiveness matrix v = Bu
 * @param Wv Diagonal entries of weighing matrix for pseudocontrol error
 * @param Wu Diagonal entries of weighing matrix for actuator penalty
 * On exit: will be normalized such that min(Wu) == 1)
 * @param n_v Length of v
 * @param n_u Length of u
 * @param theta Separation target theta^2 = min(eig(B * Wv * Wv * B^T)) / max(Wu * Wu / min(Wu * Wu))
 * @param cond_bound Upper bound of condition number of A^TA
 * @param A On exit: holds A as a colum major representation
 * @param b On exit: holds values of b
 * @param gamma On exit: holds value of gamma to respect condition number bound
 * 
 * @return nothing
*/
void setupWLS_A(
    const num_t B[AS_N_V*AS_N_U], const num_t Wv[AS_N_V], num_t Wu[AS_N_U],
    const int n_v, const int n_u,
    const num_t theta, const num_t cond_bound,
    num_t A[AS_N_C*AS_N_U], num_t* gamma);

/**
 * @brief Provides b vector to the LS problem from setupWLS_A
 * 
 * @param v Desired pseudocontrol v_des
 * @param ud Preferred ("desired") actuator u
 * @param Wv Diagonal entries of weighing matrix for pseudocontrol error
 * @param Wu_norm Normalized diagonal entries of actuator weightin matrix as
 * computed by setupWLS_A
 * @param n_v Length of v
 * @param n_u Length of u
 * @param gamma actuator-penalty deprioritizer as computed by setupWLS_A
 * @param b On exit: holds values b for the LS formulation, see setupWLS_A
 * 
*/
void setupWLS_b(
    const num_t v[AS_N_V], const num_t ud[AS_N_U],
    const num_t Wv[AS_N_V], const num_t Wu_norm[AS_N_U],
    const int n_v, const int n_u,
    const num_t gamma,
    num_t b[AS_N_C]);

/**
 * @brief Estimate lower bound for gamma, such that max condition number is met
 * 
 * Uses Gershgorin disks to compute a lower bound for gamma that is
 * guaranteed to result in a LSQ system that respects:
 *   cond(A^T A) <= cond_target
 * 
 * It has been experimentally verified (https://repository.tudelft.nl/islandora/object/uuid%3Abffb47bf-5864-4b18-921b-588b3a664866)
 * that in many cases, especially for larger platforms, the real cond(A^T A) is
 * also close to cond_target.
 * 
 * @param d Length of each dimension of A2
 * @param A2 pointer of columns of (B * Wv * Wv * B^T)
 * @param cond_target Value of the condition number to match, but not exceed
 * @param gamma On exit: holds the value of gamma as described above
 * @param max_sig On exit: holds an over-estimate of the largest eig(A^TA)
 */
void gamma_estimator(
    const int d, num_t** A2, const num_t cond_target,
    num_t* gamma, num_t* max_sig);

/**
 * @brief (Over)estimate cond(A^TA) as would be computed with setupWLS
 * 
 * Uses Gershgorin disks to compute and is essentially the inverse of 
 * gamma_estimator.
 * 
 * @param d Length of each dimension of A2
 * @param A2 pointer of columns of (B * Wv * Wv * B^T)
 * @param min_diag2 Square of the minimum value of gamma*diag(Wu)
 * @param cond_est On exit: over-estimate of condition number
 * @param max_sig On exit: holds an over-estimate of the largest eig(A^TA)
 */
void cond_estimator(
    const int d, num_t** A2, const num_t min_diag2,
    num_t* cond_est, num_t* max_sig);

#endif

