/**
 * Copyright (C) Till Blaha 2022-2023
 * MAVLab -- Faculty of Aerospace Engineering -- Delft University of Techology
 */

/**
 * @file solveActiveSet.h
 * 
 * @brief Declares variants of the active-set solver, gateway funcs and utility
*/

#ifndef SOLVEACTIVESET_H
#define SOLVEACTIVESET_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __FAST_MATH__
#error "Do not use -ffast-math, needed for protection against nan"
#endif

#ifndef AS_N_U
#error "AS_N_U must be defined."
#endif

#ifndef AS_N_V
#error "AS_N_V must be defined."
#endif

#define AS_N_C (AS_N_U+AS_N_V)

#if defined(AS_RECORD_COST) && !defined(AS_RECORD_COST_N)
#error "AS_RECORD_COST_N must be defined if AS_RECORD_COST is defined."
#endif

#if defined(AS_COST_TRUNCATE) && !defined(AS_RTOL)
#define AS_RTOL 1e-7
#endif

#if defined(AS_COST_TRUNCATE) && !defined(AS_CTOL)
#define AS_CTOL 1e-7
#endif

#ifdef AS_SINGLE_FLOAT
typedef float num_t;
#define AS_CONSTR_TOL 1e-4
#else
typedef double num_t;
#define AS_CONSTR_TOL 1e-7
#endif

typedef enum {
  AS_SUCCESS = 0,
  AS_ITER_LIMIT = 1,
  AS_COST_BELOW_TOL = 2,
  AS_COST_PLATEAU = 3,
  AS_NAN_FOUND_Q = 4,
  AS_NAN_FOUND_US = 5,
  } activeSetExitCode;

typedef enum {
  AS_QR_NAIVE = 0, // mostly previous PPRZ
  AS_QR = 1,
  AS_CHOL = 2,
#ifdef AS_INCLUDE_CG
  AS_CG = 3,
#endif
  } activeSetAlgoChoice;

/**
 * @brief Function type definition for any active-set solver
 * 
 * Solve min( ||Au-b||_2 )
 *          s.t. umin[i] <= u[i] <= umax[i]
 * 
 * @param A_col col-major representation of A
 * @param b rhs of equation
 * @param umin lower bound on u
 * @param umax upper bound on u
 * @param us initial guess. Becomes solution on exit, unless return > 0
 * @param Ws inital working set. Becomes working-set of iterate us on exit.
 * @param imax max number of iterations, default 100 if imax = 0
 * @param n_u Length of us
 * @param n_v Such that n_u+n_v is length of b
 * @param iter On exit: number of active-set iterations performed
 * @param n_free On exit: number of free variables at iterate us
 * @param costs if AS_RECORD_COST defined: cost at first AS_RECORD_COST_N iterations
 * 
 * @return 0 if success us is true solution, >0 if error. See activeSetExitCode
 */
typedef activeSetExitCode (*activeSetAlgo)(
  const num_t A_col[AS_N_C*AS_N_U], const num_t b[AS_N_C],
  const num_t umin[AS_N_U], const num_t umax[AS_N_U], num_t us[AS_N_U],
  int8_t Ws[AS_N_U], int imax, const int n_u, const int n_v,
  int *iter, int *n_free, num_t costs[]);

activeSetExitCode solveActiveSet_qr_naive(
  const num_t A_col[AS_N_C*AS_N_U], const num_t b[AS_N_C],
  const num_t umin[AS_N_U], const num_t umax[AS_N_U], num_t us[AS_N_U],
  int8_t Ws[AS_N_U], int imax, const int n_u, const int n_v,
  int *iter, int *n_free, num_t costs[]);
activeSetExitCode solveActiveSet_qr(
  const num_t A_col[AS_N_C*AS_N_U], const num_t b[AS_N_C],
  const num_t umin[AS_N_U], const num_t umax[AS_N_U], num_t us[AS_N_U],
  int8_t Ws[AS_N_U], int imax, const int n_u, const int n_v,
  int *iter, int *n_free, num_t costs[]);
activeSetExitCode solveActiveSet_chol(
  const num_t A_col[AS_N_C*AS_N_U], const num_t b[AS_N_C],
  const num_t umin[AS_N_U], const num_t umax[AS_N_U], num_t us[AS_N_U],
  int8_t Ws[AS_N_U], int imax, const int n_u, const int n_v,
  int *iter, int *n_free, num_t costs[]);
#ifdef AS_INCLUDE_CG
activeSetExitCode solveActiveSet_cg(
  const num_t A_col[AS_N_C*AS_N_U], const num_t b[AS_N_C],
  const num_t umin[AS_N_U], const num_t umax[AS_N_U], num_t us[AS_N_U],
  int8_t Ws[AS_N_U], int imax, const int n_u, const int n_v,
  int *iter, int *n_free, num_t costs[]);
#endif

/**
 * @brief Gateway function the allows switching 
 * 
 * Usage example:
 * activeSetExitCode alloc_result;
 * alloc_result = solveActiveSet(AS_QR)(A_col, b, ...);
 * 
 * @param choice Which algorithm return
 * 
 * @return pointer to a activeSetAlgo
 */
activeSetAlgo solveActiveSet(activeSetAlgoChoice choice);

/**
 * @brief Compute penalty function cost as ||Au - b||^2
 * 
 * Uses sparse multiplication to save time.
 * 
 * @param A_col col-major representation of A
 * @param b rhs of equation
 * @param u test point u
 * @param n_u number of elements in u
 * @param n_v Such that n_u + n_v is length of b
 * 
 * @return cost of quadratic penalty function at point u
 */
#ifdef AS_RECORD_COST
num_t calc_cost(const num_t A_col[AS_N_C*AS_N_U], const num_t b[AS_N_C],
  const num_t u[AS_N_U], const int n_u, const int n_v);
#endif

#endif

