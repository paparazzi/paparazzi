/**
 * Copyright (C) Till Blaha 2022-2023
 * MAVLab -- Faculty of Aerospace Engineering -- Delft University of Techology
 */

/**
 * @file sparse_math.c
 * 
 * @brief Implementations of sparse_math.h
*/

#include "sparse_math.h"
#include <stdbool.h>

void backward_tri_solve(int n, num_t** A, const num_t* b, num_t* x) {
    if (n < 1)
        return;

    x[n-1] = b[n-1] / A[n-1][n-1];

    for (int i=n-2; i >= 0; i--) {
        num_t tsum = 0;
        for (int j=i+1; j < n; j++) {
            tsum += A[i][j] * x[j];
        }

        x[i] = (b[i] - tsum) / A[i][i];
    }
}

void tri_mult(int n, int m, num_t** A, const num_t* x, num_t* b) {
    for (int i=0; i < n; i++) {
        b[i] = 0;
        for (int j=i; j < m; j++) {
            b[i] += A[i][j] * x[j];
        }
    }
}

void block_diag_self_mult(int n, int m, num_t** A, num_t** H, int s_dense, int* pos) {
    // H=A'*A

    (void)(n);

    // calculate dense part
    int i, j, k;
    for (i=0; i<m; i++) {
        for (k=0; k<=i; k++) { // smaller equal!!! <=
            H[i][k] = 0.;
            for (j=0; j<s_dense; j++) {
                H[i][k] += ((num_t) A[j][i]) * ((num_t) A[j][k]);
            }
            if (i != k)
                H[k][i] = H[i][k]; // once done, copy to transpose element
        }
    }

    for (i=0; i<m; i++) {
        H[i][i] += ((num_t) A[s_dense+pos[i]][i]) * ((num_t) A[s_dense+pos[i]][i]);
    }
}

void block_diag_mult(int n, int m, int p, num_t** A, num_t** B,
    num_t** Y, int s_dense, bool vertical, int* pos) {

    int nd = vertical ? s_dense : n;
    int md = vertical ? m : s_dense;

    // calculate dense part
    int i, j, k;
    for (k=0; k<p; k++) {
        for (i=0; i<nd; i++) {
            Y[i][k] = 0;
            for (j=0; j<md; j++) {
                Y[i][k] += A[i][j]*B[j][k];
            }
        }
    }

    // sparse part
    if (vertical) {
        for (k=0; k<p; k++) {
            for (i=0; i<n-nd; i++) {
                if (pos[i] > -1) {
                    Y[nd+i][k] = A[nd+i][pos[i]] * B[pos[i]][k];
                }
            }
        }
    } else {
        for (k=0; k<p; k++) {
            for (i=0; i<n; i++) {
                if (pos[i] > -1) {
                    Y[i][k] += A[i][md+pos[i]] * B[md+pos[i]][k];
                }
            }
        }
    }
}

int check_limits_tol(int n, num_t tol, num_t* x, const num_t* xmin, const num_t* xmax, int8_t* output, int* perm) {
    // provides relative toleranced limits checking as 
    int ind;
    int res = 0;
    for (int i=0; i<n; i++) {
        ind = perm ? perm[i] : i;
        if ( x[ind] >= (xmax[ind] * (1 + ((xmax[ind]>0) ? 1 : -1) * tol) + tol) ) {
            // violated upper bound
            output[ind] = +1;
            res++;
        } else if ( x[ind] <= (xmin[ind] * (1 + ((xmin[ind]<0) ? 1 : -1) * tol) - tol) ) {
            // violated lower bound
            output[ind] = -1;
            res++;
        } else {
            output[ind] = 0;
        }
    }

    return res;
}
