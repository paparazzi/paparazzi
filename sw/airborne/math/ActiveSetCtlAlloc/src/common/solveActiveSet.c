/**
 * Copyright (C) Till Blaha 2022-2023
 * MAVLab -- Faculty of Aerospace Engineering -- Delft University of Techology
 */

/**
 * @file solveActiveSet.c
 * 
 * @brief Implementations of solveActiveSet.h
*/

#include "solveActiveSet.h"

activeSetAlgo solveActiveSet(activeSetAlgoChoice choice) {
    switch (choice) {
        case AS_QR_NAIVE:
            return &solveActiveSet_qr_naive;
        case AS_CHOL:
            return &solveActiveSet_chol;
#ifdef AS_INCLUDE_CG
        case AS_CG:
            return &solveActiveSet_cg;
#endif
        default:
            return &solveActiveSet_qr;
    }
};

#ifdef AS_RECORD_COST
num_t calc_cost(const num_t A_col[AS_N_C*AS_N_U], const num_t b[AS_N_C],
  const num_t u[AS_N_U], const int n_u, const int n_v)
{
	// checking cost in n_v*n_u+n_u time
	num_t cost = 0.;
	for (int i=0; i<(n_u+n_v); i++) {
		num_t i_cost = -b[i];
		if (i < n_v) {
			// dense part
			for (int j=0; j<n_u; j++)
				i_cost += A_col[i + j*(n_u+n_v)]*u[j];
		} else {
			// sparse part
			i_cost += A_col[i + (i-n_v)*(n_u+n_v)]*u[i-n_v]; 
		}
		cost += i_cost*i_cost;
	}
	return cost;
}
#endif
