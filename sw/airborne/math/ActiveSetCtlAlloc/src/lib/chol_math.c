#include "chol_math.h"
#include <stdio.h>

#include <string.h>
#include <math.h>
#include <stdint.h>

void pprz_cholesky_float(num_t **out, num_t **in, num_t* inv_diag, int n)
{
  int i, j, k;

  for (i = 0; i < n; i++) {
    for (j = 0; j < (i + 1); j++) {
      num_t s = 0;
      for (k = 0; k < j; k++) {
        s += out[i][k] * out[j][k];
      }
      if (i == j) {
        out[i][j] = sqrt(in[i][i] - s);
        inv_diag[j] = (out[i][j] != 0) ? (1.0 / out[i][j]) : 0.0;
      } else {
        out[i][j] = inv_diag[j] * (in[i][j] - s);
      }
    }
  }
}

void cholesky_solve(num_t **L, num_t* inv_diag, int n, num_t *b, num_t *x) {
    // Antoine Drouin, 2007, modified
	int j,k;
	num_t t;

    for(j = 0 ; j < n ; j++) { // solve Ly=b
        t = b[j];
        for(k = j - 1 ; k >= 0 ; k--)
            t -= L[j][k] * x[k];
        x[j] = t*inv_diag[j];
    }
    for(j = n - 1 ; j >= 0 ; j--) { // solve Ltx=y
        t = x[j];
        for(k = j + 1 ; k < n ; k++)
            t -= L[k][j] * x[k];
        x[j] = t*inv_diag[j];
    }
}

void cholup(num_t** L, int i1, int i2)
{
#ifdef AS_VERBOSE
    if (i1 < 1)
        printf("should not happen");
#endif

    for (int k=i1; k<i2; k++) {
        num_t r;
        r = hypot(L[k][k], L[k][i1-1]); // (a**2 + b**2)^.5 avoiding overflows
        num_t Lkkinv = 1.0 / L[k][k];
        num_t c = r * Lkkinv;
        num_t cinv = 1/c;
        num_t s = L[k][i1-1] * Lkkinv;
        L[k][k] = r;

        if (k < i2-1) {
            for (int j=k; j<i2-1; j++) {
                L[j+1][k] += s * L[j+1][i1-1];
                L[j+1][k] *= cinv;
                L[j+1][i1-1] *= c;
                L[j+1][i1-1] -= s * L[j+1][k];
            }
        }

    }
}

void choldel(num_t** L, num_t* inv_diag, int f, int n)
{
    cholup(L, f+1, n);

    for (int i=f; i<n-1; i++) {
        for (int j=0; j<f; j++) {
            L[i][j] = L[i+1][j];
        }
        for (int j=f; j<n-1; j++) {
            L[i][j] = L[i+1][j+1];
            if (i == j)
                inv_diag[i] = 1 / L[i][i]; // potential for optimisation by assigning this inside cholup? Don't think so
        }
    }
}

void choladd(num_t** L, int nf, num_t* inv_diag, num_t** H, int* perm, int f)
{

    num_t t;
    for (int j = 0; j < nf; j++) { // solve L11 L21' = H[perm[1:nf]][perm[f]]
        t = H[perm[j]][perm[f]];
        for (int k = j-1; k >=0; k--)
            t -= L[j][k] * L[nf][k];
        L[nf][j] = t / L[j][j];
    }

    num_t L21L21T = 0.0;
    for (int j = 0; j < nf; j++) {
        L21L21T += L[nf][j]*L[nf][j];
    }

    L[nf][nf] = sqrt(H[perm[f]][perm[f]] - L21L21T);
    inv_diag[nf] = 1.0 / L[nf][nf];
}

