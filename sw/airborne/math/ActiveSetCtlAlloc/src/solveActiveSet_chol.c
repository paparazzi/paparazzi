#include "solveActiveSet.h"
#include "setupWLS.h"
#include "chol_math.h"
#include "sparse_math.h"
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <stdbool.h>

activeSetExitCode solveActiveSet_chol(
  const num_t A_col[AS_N_C*AS_N_U], const num_t b[AS_N_C],
  const num_t umin[AS_N_U], const num_t umax[AS_N_U], num_t us[AS_N_U],
  int8_t Ws[AS_N_U], int imax, const int n_u, const int n_v,
  int *iter, int *n_free, num_t costs[])
{

#ifndef AS_RECORD_COST
  (void) costs;
#endif

  if(!imax) imax = 100;

  activeSetExitCode exit_code = AS_ITER_LIMIT;

  int n_c = n_u + n_v;
  uint8_t i;
  uint8_t j;

  for (i = 0; i < n_u; i++) {
    if (Ws[i] == 0) {
      us[i] = (us[i] > umax[i]) ? umax[i] : ((us[i] < umin[i]) ? umin[i] : us[i]);
    } else {
      us[i] = (Ws[i] > 0) ? umax[i] : umin[i];
    }
  }

  num_t A[AS_N_C][AS_N_U];
  num_t H_perm[AS_N_U][AS_N_U];
  num_t H[AS_N_U][AS_N_U];
  num_t L[AS_N_U][AS_N_U];

  // Create a pointer array to the rows of A
  // such that we can pass it to a function
  num_t * A_ptr[AS_N_C];
  num_t * H_perm_ptr[AS_N_U];
  num_t * H_ptr[AS_N_U];
  num_t * L_ptr[AS_N_U];
  for(i = 0; i < n_c; i++) {
    A_ptr[i] = A[i];
    if (i < n_u) { H_perm_ptr[i] = H_perm[i]; H_ptr[i] = H[i]; L_ptr[i] = L[i]; }
  }

  int permutation[AS_N_U]; memset(permutation, 0, sizeof(int)*n_u);
  (*n_free) = 0;
  uint8_t i_bnd = 0;
  for (i = 0; i < n_u; i++) {
    if (Ws[i] == 0) {
      permutation[(*n_free)++] = i;
    }
  }
  for (i = 0; i < n_u; i++) {
    if (Ws[i] != 0) {
      permutation[(i_bnd++)+(*n_free)] = i;
    }
  }

  // convert col major input to 2d array
  for(i = 0; i < n_c; i++) {
    for(j = 0; j < n_u; j++) {
      A[i][j] = A_col[i + n_c * j];
    }
  }

  // initial factorisation
  int dummy[AS_N_U];
  for (i = 0; i < n_u; i++)
    dummy[i] = i;

  block_diag_self_mult(n_c, n_u, A_ptr, H_ptr, n_v, dummy);
  for(i = 0; i < n_u; i++) {
    for(j = 0; j < n_u; j++) {
      H_perm[i][j] = H[permutation[i]][permutation[j]];
    }
  }
  
  // debug output
  #ifdef AS_VERBOSE
  printf("H:\n");
  for (i =0; i<n_u; i++) {
    for (j =0; j<n_u; j++) {
      printf("%f ", H_ptr[i][j]);
    }
    printf("\n");
  }
  printf("\n");
  #endif

  num_t inv_diag[AS_N_U];
  pprz_cholesky_float(L_ptr, H_perm_ptr, inv_diag, n_u);

  // debug output
  #ifdef AS_VERBOSE
  printf("H_perm:\n");
  for (i =0; i<n_u; i++) {
    for (j =0; j<n_u; j++) {
      printf("%f ", H_perm_ptr[i][j]);
    }
    printf("\n");
  }
  printf("\n");
  #endif


  num_t q[AS_N_U];
  num_t z[AS_N_U];
  bool nan_found = false;

  // -------------- Start loop ------------
  *iter = 0;
#ifdef AS_COST_TRUNCATE
  num_t prev_cost = INFINITY;
#endif
  while (++(*iter) <= imax) {
    num_t beta[AS_N_U];
    for (i=0; i<(*n_free); i++) {
      beta[i] = 0;
      for (j=(*n_free); j<n_u; j++)
        beta[i] -= H[permutation[i]][permutation[j]] * us[permutation[j]];

      // beta += A'*b, but is optimised, because A has diagonal part
      // TODO: can be pre-computed!! No need to do every iteration
      for (j=0; j<n_v; j++)
        beta[i] += A[j][permutation[i]] * b[j];

      beta[i] += A[n_v+permutation[i]][permutation[i]] * b[n_v+permutation[i]];

      #ifdef AS_VERBOSE
      printf("%f\n", beta[i]);
      #endif
    }

    #ifdef AS_VERBOSE
    printf("L:\n");
    for (i =0; i<n_u; i++) {
      for (j =0; j<n_u; j++) {
        printf("%f ", L_ptr[i][j]);
      }
      printf("\n");
    }
    printf("\n");
    #endif
    cholesky_solve(L_ptr, inv_diag, (*n_free), beta, q);

    for (i = 0; i < (*n_free); i++) {
      // check for nan according to IEEE 754 assuming -ffast-math is not passed
      if (q[i] != q[i]) {
        // break immediately with error
        nan_found = true;
        break;
      }
      z[permutation[i]] = q[i];
    }
    if (nan_found) {
      exit_code = AS_NAN_FOUND_Q;
      break;
    }
    for (i = (*n_free); i < n_u; i++) {
      z[permutation[i]] = us[permutation[i]];
    }

    uint8_t n_violated = 0;
    int8_t W_temp[AS_N_U];
    n_violated = check_limits_tol((*n_free), AS_CONSTR_TOL, z, umin, umax, W_temp, permutation);

    if (!n_violated) {
      for (i = 0; i < (*n_free); i++) {
        us[permutation[i]] = z[permutation[i]];
      }

      if ((*n_free) == n_u) {
        // no active constraints, we are optinal and feasible
#ifdef AS_RECORD_COST
        if ((*iter) <= AS_RECORD_COST_N)
          costs[(*iter)-1] = calc_cost(A_col, b, us, n_u, n_v);
#endif
        exit_code = AS_SUCCESS;
        break;
      } else {
        // active constraints, check for optimality

        num_t lambda_perm[AS_N_U];
        uint8_t f_free = 0;
        num_t maxlam = -INFINITY;

        num_t r[AS_N_C];
        num_t r_sq = 0.;
        // dense part
        for (i = 0; i<n_v; i++) {
          r[i] = -b[i];
          for (j =0; j<n_u; j++)
            r[i] += A[i][j]*z[j];
          r_sq += r[i]*r[i];
        }
        // diagonal part
        for (i=n_v; i<n_c; i++) {
          r[i] = -b[i] + A[i][i-n_v]*z[i-n_v];
          r_sq += r[i]*r[i];
        }

        // check cost
#ifdef AS_COST_TRUNCATE
        if (r_sq <= AS_CTOL) {
          exit_code = AS_COST_BELOW_TOL;
          break;
        }
        num_t diff = prev_cost - r_sq;
        if ((diff < 0.) || (diff/prev_cost < AS_RTOL)) {
          exit_code = AS_COST_PLATEAU;
          break;
        }
        prev_cost = r_sq;
#endif

        for (i = (*n_free); i<n_u; i++) {
          lambda_perm[i] = 0;
          // lambda = A^T*r but is optimised with the diagonal part of A
          // dense part
          for (j = 0; j < n_v; j++)
            lambda_perm[i] -= A[j][permutation[i]]*r[j];

          // diagonal part
          lambda_perm[i] -= A[permutation[i]+n_v][permutation[i]]*r[permutation[i]+n_v];

          lambda_perm[i] *= -Ws[permutation[i]];
          if (lambda_perm[i] > maxlam) {
            maxlam = lambda_perm[i];
            f_free = i-(*n_free);
          }
        }

        if (maxlam <= AS_CONSTR_TOL) {
#ifdef AS_RECORD_COST
          if ((*iter) <= AS_RECORD_COST_N)
            costs[(*iter)-1] = calc_cost(A_col, b, us, n_u, n_v);
#endif
          exit_code = AS_SUCCESS;
          break; // constraints hit, but optimal
        }

        // update chol
        choladd(L_ptr, (*n_free), inv_diag, H_ptr, permutation, (*n_free)+f_free);

        Ws[permutation[(*n_free)+f_free]] = 0;
        uint8_t last_val = permutation[(*n_free)+f_free];
        for (i = f_free; i > 0; i--) {
          permutation[(*n_free)+i] = permutation[(*n_free)+i-1];
        }
        permutation[(*n_free)++] = last_val;

      }
    } else {

      num_t a = INFINITY;
      uint8_t i_a = 0;
      uint8_t f_bound = 0;
      int8_t i_s = 0;
      num_t temp;
      int8_t temp_s;
      for (uint8_t f=0; f < (*n_free); f++) {
        i = permutation[f];
        if (W_temp[i] == -1) {
          temp = (us[i] - umin[i]) / (us[i] - z[i]);
          temp_s = -1;
        } else if (W_temp[i] == +1) {
          temp = (umax[i] - us[i]) / (z[i] - us[i]);
          temp_s = +1;
        } else {
          continue;
        }
        if (temp < a) {
          a = temp;
          i_a = i;
          f_bound = f;
          i_s = temp_s;
        }
      }

      // update xs
      for (i =0; i<n_u; i++) {
        num_t p = z[i] - us[i];
        num_t incr = a * p;
        if (i == i_a) {
          us[i] = (i_s == +1) ? umax[i] : umin[i];
        } else {
          us[i] += incr;
        }
        if (us[i] != us[i]) {
          // nan found
          nan_found = true;
          break;
        }
      }
      if (nan_found) {
        exit_code = AS_NAN_FOUND_US;
        break;
      }

      // update chol
      choldel(L_ptr, inv_diag, f_bound, (*n_free));

      Ws[i_a] = i_s;
      uint8_t first_val = permutation[f_bound];
      for (i = 0; i < (*n_free)-f_bound-1; i++) {
        permutation[f_bound+i] = permutation[f_bound+i+1];
      }
      permutation[--(*n_free)] = first_val;
    }

#ifdef AS_RECORD_COST
    if ((*iter) <= AS_RECORD_COST_N)
      costs[(*iter)-1] = calc_cost(A_col, b, us, n_u, n_v);
#endif

  }
  if (exit_code == AS_ITER_LIMIT)
    (*iter)--;

  return exit_code;
}
