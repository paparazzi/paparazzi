/*
 * File: UKF_Wind_Estimator.c
 *
 * Code generated for Simulink model 'UKF_Wind_Estimator'.
 *
 * Model version                  : 1.120
 * Simulink Coder version         : 8.10 (R2016a) 10-Feb-2016
 * C/C++ source code generated on : Wed Nov  2 23:49:42 2016
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: Custom Processor->Custom
 * Code generation objectives:
 *    1. Execution efficiency
 *    2. RAM efficiency
 * Validation result: Not run
 */

#include "UKF_Wind_Estimator.h"

/* Exported data definition */

/* Definition for custom storage class: Struct */
ukf_init_type ukf_init;
ukf_params_type ukf_params;

/* Block signals and states (auto storage) */
DW ukf_DW;

/* External inputs (root inport signals with auto storage) */
ExtU ukf_U;

/* External outputs (root outports fed by signals with auto storage) */
ExtY ukf_Y;

/* Forward declaration for local functions */
static real32_T xnrm2_f(int32_T n, const real32_T x[147], int32_T ix0);
static void xgeqrf_f(real32_T A[147], real32_T tau[7]);
static void qr_e(const real32_T A[147], real32_T Q[147], real32_T R[49]);
static real32_T xnrm2(int32_T n, const real32_T x[120], int32_T ix0);
static void xgeqrf(real32_T A[120], real32_T tau[6]);
static void qr(const real32_T A[120], real32_T Q[120], real32_T R[36]);
static void mrdivide(real32_T A[42], const real32_T B_0[36]);
static void h(const real32_T x[7], const real32_T q[4], real32_T y[6]);
static void RungeKutta(const real32_T x[7], real32_T dt, const real32_T u[6],
  real32_T xi[7]);

/* Function for MATLAB Function: '<Root>/UKF_prediction' */
static real32_T xnrm2_f(int32_T n, const real32_T x[147], int32_T ix0)
{
  real32_T y;
  real32_T scale;
  int32_T kend;
  real32_T absxk;
  real32_T t;
  int32_T k;
  y = 0.0F;
  if (!(n < 1)) {
    if (n == 1) {
      y = fabsf(x[ix0 - 1]);
    } else {
      scale = 1.17549435E-38F;
      kend = (ix0 + n) - 1;
      for (k = ix0; k <= kend; k++) {
        absxk = fabsf(x[k - 1]);
        if (absxk > scale) {
          t = scale / absxk;
          y = y * t * t + 1.0F;
          scale = absxk;
        } else {
          t = absxk / scale;
          y += t * t;
        }
      }

      y = scale * sqrtf(y);
    }
  }

  return y;
}

/* Function for MATLAB Function: '<Root>/UKF_prediction' */
static void xgeqrf_f(real32_T A[147], real32_T tau[7])
{
  real32_T work[7];
  int32_T i_i;
  real32_T b_atmp;
  real32_T xnorm;
  int32_T knt;
  int32_T lastv;
  int32_T lastc;
  int32_T ix;
  real32_T b_c;
  int32_T iy;
  int32_T f;
  int32_T g;
  int32_T b_ia;
  int32_T b_ix;
  boolean_T exitg2;
  int32_T i;
  for (i = 0; i < 7; i++) {
    work[i] = 0.0F;
  }

  for (i = 0; i < 7; i++) {
    i_i = i * 21 + i;
    b_atmp = A[i_i];
    b_c = 0.0F;
    xnorm = xnrm2_f(20 - i, A, i_i + 2);
    if (xnorm != 0.0F) {
      xnorm = hypotf(A[i_i], xnorm);
      if (A[i_i] >= 0.0F) {
        xnorm = -xnorm;
      }

      if (fabsf(xnorm) < 9.86076132E-32F) {
        knt = 0;
        do {
          knt++;
          lastv = (i_i - i) + 21;
          for (lastc = i_i + 1; lastc + 1 <= lastv; lastc++) {
            A[lastc] *= 1.01412048E+31F;
          }

          xnorm *= 1.01412048E+31F;
          b_atmp *= 1.01412048E+31F;
        } while (!(fabsf(xnorm) >= 9.86076132E-32F));

        xnorm = hypotf(b_atmp, xnrm2_f(20 - i, A, i_i + 2));
        if (b_atmp >= 0.0F) {
          xnorm = -xnorm;
        }

        b_c = (xnorm - b_atmp) / xnorm;
        b_atmp = 1.0F / (b_atmp - xnorm);
        lastv = (i_i - i) + 21;
        for (lastc = i_i + 1; lastc + 1 <= lastv; lastc++) {
          A[lastc] *= b_atmp;
        }

        for (lastv = 1; lastv <= knt; lastv++) {
          xnorm *= 9.86076132E-32F;
        }

        b_atmp = xnorm;
      } else {
        b_c = (xnorm - A[i_i]) / xnorm;
        b_atmp = 1.0F / (A[i_i] - xnorm);
        knt = (i_i - i) + 21;
        for (lastv = i_i + 1; lastv + 1 <= knt; lastv++) {
          A[lastv] *= b_atmp;
        }

        b_atmp = xnorm;
      }
    }

    tau[i] = b_c;
    A[i_i] = b_atmp;
    if (i + 1 < 7) {
      b_atmp = A[i_i];
      A[i_i] = 1.0F;
      knt = (i + 1) * 21 + i;
      if (tau[i] != 0.0F) {
        lastv = 21 - i;
        lastc = (i_i - i) + 20;
        while ((lastv > 0) && (A[lastc] == 0.0F)) {
          lastv--;
          lastc--;
        }

        lastc = 6 - i;
        exitg2 = false;
        while ((!exitg2) && (lastc > 0)) {
          iy = ((lastc - 1) * 21 + knt) + 1;
          f = iy;
          do {
            b_ix = 0;
            if (f <= (iy + lastv) - 1) {
              if (A[f - 1] != 0.0F) {
                b_ix = 1;
              } else {
                f++;
              }
            } else {
              lastc--;
              b_ix = 2;
            }
          } while (b_ix == 0);

          if (b_ix == 1) {
            exitg2 = true;
          }
        }
      } else {
        lastv = 0;
        lastc = 0;
      }

      if (lastv > 0) {
        if (lastc != 0) {
          for (iy = 1; iy <= lastc; iy++) {
            work[iy - 1] = 0.0F;
          }

          iy = 0;
          f = ((lastc - 1) * 21 + knt) + 1;
          for (b_ix = knt + 1; b_ix <= f; b_ix += 21) {
            ix = i_i;
            b_c = 0.0F;
            g = (b_ix + lastv) - 1;
            for (b_ia = b_ix; b_ia <= g; b_ia++) {
              b_c += A[b_ia - 1] * A[ix];
              ix++;
            }

            work[iy] += b_c;
            iy++;
          }
        }

        if (!(-tau[i] == 0.0F)) {
          iy = 0;
          for (f = 1; f <= lastc; f++) {
            if (work[iy] != 0.0F) {
              b_c = work[iy] * -tau[i];
              b_ix = i_i;
              ix = lastv + knt;
              for (g = knt; g + 1 <= ix; g++) {
                A[g] += A[b_ix] * b_c;
                b_ix++;
              }
            }

            iy++;
            knt += 21;
          }
        }
      }

      A[i_i] = b_atmp;
    }
  }
}

/* Function for MATLAB Function: '<Root>/UKF_prediction' */
static void qr_e(const real32_T A[147], real32_T Q[147], real32_T R[49])
{
  real32_T tau[7];
  int32_T c_i;
  int32_T itau;
  real32_T work[7];
  int32_T iaii;
  int32_T lastv;
  int32_T lastc;
  int32_T ix;
  real32_T c;
  int32_T iy;
  int32_T iac;
  int32_T d;
  int32_T b_ia;
  int32_T jy;
  boolean_T exitg2;
  memcpy(&Q[0], &A[0], 147U * sizeof(real32_T));
  xgeqrf_f(Q, tau);
  for (itau = 0; itau < 7; itau++) {
    for (c_i = 0; c_i + 1 <= itau + 1; c_i++) {
      R[c_i + 7 * itau] = Q[21 * itau + c_i];
    }

    for (c_i = itau + 1; c_i + 1 < 8; c_i++) {
      R[c_i + 7 * itau] = 0.0F;
    }

    work[itau] = 0.0F;
  }

  itau = 6;
  for (c_i = 6; c_i >= 0; c_i += -1) {
    iaii = (c_i * 21 + c_i) + 1;
    if (c_i + 1 < 7) {
      Q[iaii - 1] = 1.0F;
      if (tau[itau] != 0.0F) {
        lastv = 21 - c_i;
        lastc = (iaii - c_i) + 19;
        while ((lastv > 0) && (Q[lastc] == 0.0F)) {
          lastv--;
          lastc--;
        }

        lastc = 6 - c_i;
        exitg2 = false;
        while ((!exitg2) && (lastc > 0)) {
          iy = (lastc - 1) * 21 + iaii;
          jy = iy;
          do {
            iac = 0;
            if (jy + 21 <= (iy + lastv) + 20) {
              if (Q[jy + 20] != 0.0F) {
                iac = 1;
              } else {
                jy++;
              }
            } else {
              lastc--;
              iac = 2;
            }
          } while (iac == 0);

          if (iac == 1) {
            exitg2 = true;
          }
        }
      } else {
        lastv = 0;
        lastc = 0;
      }

      if (lastv > 0) {
        if (lastc != 0) {
          for (iy = 1; iy <= lastc; iy++) {
            work[iy - 1] = 0.0F;
          }

          iy = 0;
          jy = ((lastc - 1) * 21 + iaii) + 21;
          for (iac = iaii + 21; iac <= jy; iac += 21) {
            ix = iaii;
            c = 0.0F;
            d = (iac + lastv) - 1;
            for (b_ia = iac; b_ia <= d; b_ia++) {
              c += Q[b_ia - 1] * Q[ix - 1];
              ix++;
            }

            work[iy] += c;
            iy++;
          }
        }

        if (!(-tau[itau] == 0.0F)) {
          iy = iaii + 20;
          jy = 0;
          for (iac = 1; iac <= lastc; iac++) {
            if (work[jy] != 0.0F) {
              c = work[jy] * -tau[itau];
              ix = iaii;
              d = lastv + iy;
              for (b_ia = iy; b_ia + 1 <= d; b_ia++) {
                Q[b_ia] += Q[ix - 1] * c;
                ix++;
              }
            }

            jy++;
            iy += 21;
          }
        }
      }
    }

    lastv = (iaii - c_i) + 20;
    for (lastc = iaii; lastc + 1 <= lastv; lastc++) {
      Q[lastc] *= -tau[itau];
    }

    Q[iaii - 1] = 1.0F - tau[itau];
    for (lastv = 1; lastv <= c_i; lastv++) {
      Q[(iaii - lastv) - 1] = 0.0F;
    }

    itau--;
  }
}

/* Function for MATLAB Function: '<Root>/UKF_correction' */
static real32_T xnrm2(int32_T n, const real32_T x[120], int32_T ix0)
{
  real32_T y;
  real32_T scale;
  int32_T kend;
  real32_T absxk;
  real32_T t;
  int32_T k;
  y = 0.0F;
  if (!(n < 1)) {
    if (n == 1) {
      y = fabsf(x[ix0 - 1]);
    } else {
      scale = 1.17549435E-38F;
      kend = (ix0 + n) - 1;
      for (k = ix0; k <= kend; k++) {
        absxk = fabsf(x[k - 1]);
        if (absxk > scale) {
          t = scale / absxk;
          y = y * t * t + 1.0F;
          scale = absxk;
        } else {
          t = absxk / scale;
          y += t * t;
        }
      }

      y = scale * sqrtf(y);
    }
  }

  return y;
}

/* Function for MATLAB Function: '<Root>/UKF_correction' */
static void xgeqrf(real32_T A[120], real32_T tau[6])
{
  real32_T work[6];
  int32_T i_i;
  real32_T b_atmp;
  real32_T xnorm;
  int32_T knt;
  int32_T lastv;
  int32_T lastc;
  int32_T ix;
  real32_T b_c;
  int32_T iy;
  int32_T f;
  int32_T g;
  int32_T b_ia;
  int32_T b_ix;
  boolean_T exitg2;
  int32_T i;
  for (i = 0; i < 6; i++) {
    work[i] = 0.0F;
  }

  for (i = 0; i < 6; i++) {
    i_i = i * 20 + i;
    b_atmp = A[i_i];
    b_c = 0.0F;
    xnorm = xnrm2(19 - i, A, i_i + 2);
    if (xnorm != 0.0F) {
      xnorm = hypotf(A[i_i], xnorm);
      if (A[i_i] >= 0.0F) {
        xnorm = -xnorm;
      }

      if (fabsf(xnorm) < 9.86076132E-32F) {
        knt = 0;
        do {
          knt++;
          lastv = (i_i - i) + 20;
          for (lastc = i_i + 1; lastc + 1 <= lastv; lastc++) {
            A[lastc] *= 1.01412048E+31F;
          }

          xnorm *= 1.01412048E+31F;
          b_atmp *= 1.01412048E+31F;
        } while (!(fabsf(xnorm) >= 9.86076132E-32F));

        xnorm = hypotf(b_atmp, xnrm2(19 - i, A, i_i + 2));
        if (b_atmp >= 0.0F) {
          xnorm = -xnorm;
        }

        b_c = (xnorm - b_atmp) / xnorm;
        b_atmp = 1.0F / (b_atmp - xnorm);
        lastv = (i_i - i) + 20;
        for (lastc = i_i + 1; lastc + 1 <= lastv; lastc++) {
          A[lastc] *= b_atmp;
        }

        for (lastv = 1; lastv <= knt; lastv++) {
          xnorm *= 9.86076132E-32F;
        }

        b_atmp = xnorm;
      } else {
        b_c = (xnorm - A[i_i]) / xnorm;
        b_atmp = 1.0F / (A[i_i] - xnorm);
        knt = (i_i - i) + 20;
        for (lastv = i_i + 1; lastv + 1 <= knt; lastv++) {
          A[lastv] *= b_atmp;
        }

        b_atmp = xnorm;
      }
    }

    tau[i] = b_c;
    A[i_i] = b_atmp;
    if (i + 1 < 6) {
      b_atmp = A[i_i];
      A[i_i] = 1.0F;
      knt = (i + 1) * 20 + i;
      if (tau[i] != 0.0F) {
        lastv = 20 - i;
        lastc = (i_i - i) + 19;
        while ((lastv > 0) && (A[lastc] == 0.0F)) {
          lastv--;
          lastc--;
        }

        lastc = 5 - i;
        exitg2 = false;
        while ((!exitg2) && (lastc > 0)) {
          iy = ((lastc - 1) * 20 + knt) + 1;
          f = iy;
          do {
            b_ix = 0;
            if (f <= (iy + lastv) - 1) {
              if (A[f - 1] != 0.0F) {
                b_ix = 1;
              } else {
                f++;
              }
            } else {
              lastc--;
              b_ix = 2;
            }
          } while (b_ix == 0);

          if (b_ix == 1) {
            exitg2 = true;
          }
        }
      } else {
        lastv = 0;
        lastc = 0;
      }

      if (lastv > 0) {
        if (lastc != 0) {
          for (iy = 1; iy <= lastc; iy++) {
            work[iy - 1] = 0.0F;
          }

          iy = 0;
          f = ((lastc - 1) * 20 + knt) + 1;
          for (b_ix = knt + 1; b_ix <= f; b_ix += 20) {
            ix = i_i;
            b_c = 0.0F;
            g = (b_ix + lastv) - 1;
            for (b_ia = b_ix; b_ia <= g; b_ia++) {
              b_c += A[b_ia - 1] * A[ix];
              ix++;
            }

            work[iy] += b_c;
            iy++;
          }
        }

        if (!(-tau[i] == 0.0F)) {
          iy = 0;
          for (f = 1; f <= lastc; f++) {
            if (work[iy] != 0.0F) {
              b_c = work[iy] * -tau[i];
              b_ix = i_i;
              ix = lastv + knt;
              for (g = knt; g + 1 <= ix; g++) {
                A[g] += A[b_ix] * b_c;
                b_ix++;
              }
            }

            iy++;
            knt += 20;
          }
        }
      }

      A[i_i] = b_atmp;
    }
  }
}

/* Function for MATLAB Function: '<Root>/UKF_correction' */
static void qr(const real32_T A[120], real32_T Q[120], real32_T R[36])
{
  real32_T tau[6];
  int32_T c_i;
  int32_T itau;
  real32_T work[6];
  int32_T iaii;
  int32_T lastv;
  int32_T lastc;
  int32_T ix;
  real32_T c;
  int32_T iy;
  int32_T iac;
  int32_T d;
  int32_T b_ia;
  int32_T jy;
  boolean_T exitg2;
  memcpy(&Q[0], &A[0], 120U * sizeof(real32_T));
  xgeqrf(Q, tau);
  for (itau = 0; itau < 6; itau++) {
    for (c_i = 0; c_i + 1 <= itau + 1; c_i++) {
      R[c_i + 6 * itau] = Q[20 * itau + c_i];
    }

    for (c_i = itau + 1; c_i + 1 < 7; c_i++) {
      R[c_i + 6 * itau] = 0.0F;
    }

    work[itau] = 0.0F;
  }

  itau = 5;
  for (c_i = 5; c_i >= 0; c_i += -1) {
    iaii = (c_i * 20 + c_i) + 1;
    if (c_i + 1 < 6) {
      Q[iaii - 1] = 1.0F;
      if (tau[itau] != 0.0F) {
        lastv = 20 - c_i;
        lastc = (iaii - c_i) + 18;
        while ((lastv > 0) && (Q[lastc] == 0.0F)) {
          lastv--;
          lastc--;
        }

        lastc = 5 - c_i;
        exitg2 = false;
        while ((!exitg2) && (lastc > 0)) {
          iy = (lastc - 1) * 20 + iaii;
          jy = iy;
          do {
            iac = 0;
            if (jy + 20 <= (iy + lastv) + 19) {
              if (Q[jy + 19] != 0.0F) {
                iac = 1;
              } else {
                jy++;
              }
            } else {
              lastc--;
              iac = 2;
            }
          } while (iac == 0);

          if (iac == 1) {
            exitg2 = true;
          }
        }
      } else {
        lastv = 0;
        lastc = 0;
      }

      if (lastv > 0) {
        if (lastc != 0) {
          for (iy = 1; iy <= lastc; iy++) {
            work[iy - 1] = 0.0F;
          }

          iy = 0;
          jy = ((lastc - 1) * 20 + iaii) + 20;
          for (iac = iaii + 20; iac <= jy; iac += 20) {
            ix = iaii;
            c = 0.0F;
            d = (iac + lastv) - 1;
            for (b_ia = iac; b_ia <= d; b_ia++) {
              c += Q[b_ia - 1] * Q[ix - 1];
              ix++;
            }

            work[iy] += c;
            iy++;
          }
        }

        if (!(-tau[itau] == 0.0F)) {
          iy = iaii + 19;
          jy = 0;
          for (iac = 1; iac <= lastc; iac++) {
            if (work[jy] != 0.0F) {
              c = work[jy] * -tau[itau];
              ix = iaii;
              d = lastv + iy;
              for (b_ia = iy; b_ia + 1 <= d; b_ia++) {
                Q[b_ia] += Q[ix - 1] * c;
                ix++;
              }
            }

            jy++;
            iy += 20;
          }
        }
      }
    }

    lastv = (iaii - c_i) + 19;
    for (lastc = iaii; lastc + 1 <= lastv; lastc++) {
      Q[lastc] *= -tau[itau];
    }

    Q[iaii - 1] = 1.0F - tau[itau];
    for (lastv = 1; lastv <= c_i; lastv++) {
      Q[(iaii - lastv) - 1] = 0.0F;
    }

    itau--;
  }
}

/* Function for MATLAB Function: '<Root>/main' */
static void mrdivide(real32_T A[42], const real32_T B_0[36])
{
  int32_T jp;
  real32_T temp;
  real32_T b_A[36];
  int8_T ipiv[6];
  int32_T j;
  int32_T ix;
  real32_T s;
  int32_T c_ix;
  int32_T d;
  int32_T ijA;
  int32_T b_jAcol;
  int32_T b_kBcol;
  memcpy(&b_A[0], &B_0[0], 36U * sizeof(real32_T));
  for (j = 0; j < 6; j++) {
    ipiv[j] = (int8_T)(1 + j);
  }

  for (j = 0; j < 5; j++) {
    jp = j * 7;
    b_jAcol = 0;
    ix = jp;
    temp = fabsf(b_A[jp]);
    for (b_kBcol = 2; b_kBcol <= 6 - j; b_kBcol++) {
      ix++;
      s = fabsf(b_A[ix]);
      if (s > temp) {
        b_jAcol = b_kBcol - 1;
        temp = s;
      }
    }

    if (b_A[jp + b_jAcol] != 0.0F) {
      if (b_jAcol != 0) {
        ipiv[j] = (int8_T)((j + b_jAcol) + 1);
        ix = j;
        b_jAcol += j;
        for (b_kBcol = 0; b_kBcol < 6; b_kBcol++) {
          temp = b_A[ix];
          b_A[ix] = b_A[b_jAcol];
          b_A[b_jAcol] = temp;
          ix += 6;
          b_jAcol += 6;
        }
      }

      b_jAcol = (jp - j) + 6;
      for (ix = jp + 1; ix + 1 <= b_jAcol; ix++) {
        b_A[ix] /= b_A[jp];
      }
    }

    b_jAcol = jp;
    ix = jp + 6;
    for (b_kBcol = 1; b_kBcol <= 5 - j; b_kBcol++) {
      temp = b_A[ix];
      if (b_A[ix] != 0.0F) {
        c_ix = jp + 1;
        d = (b_jAcol - j) + 12;
        for (ijA = 7 + b_jAcol; ijA + 1 <= d; ijA++) {
          b_A[ijA] += b_A[c_ix] * -temp;
          c_ix++;
        }
      }

      ix += 6;
      b_jAcol += 6;
    }
  }

  for (j = 0; j < 6; j++) {
    jp = 7 * j;
    b_jAcol = 6 * j;
    for (ix = 1; ix <= j; ix++) {
      b_kBcol = (ix - 1) * 7;
      if (b_A[(ix + b_jAcol) - 1] != 0.0F) {
        for (c_ix = 0; c_ix < 7; c_ix++) {
          A[c_ix + jp] -= b_A[(ix + b_jAcol) - 1] * A[c_ix + b_kBcol];
        }
      }
    }

    temp = 1.0F / b_A[j + b_jAcol];
    for (b_jAcol = 0; b_jAcol < 7; b_jAcol++) {
      A[b_jAcol + jp] *= temp;
    }
  }

  for (j = 5; j >= 0; j += -1) {
    jp = 7 * j;
    b_jAcol = 6 * j - 1;
    for (ix = j + 2; ix < 7; ix++) {
      b_kBcol = (ix - 1) * 7;
      if (b_A[ix + b_jAcol] != 0.0F) {
        for (c_ix = 0; c_ix < 7; c_ix++) {
          A[c_ix + jp] -= b_A[ix + b_jAcol] * A[c_ix + b_kBcol];
        }
      }
    }
  }

  for (j = 4; j >= 0; j += -1) {
    if (j + 1 != ipiv[j]) {
      jp = ipiv[j] - 1;
      for (b_jAcol = 0; b_jAcol < 7; b_jAcol++) {
        temp = A[7 * j + b_jAcol];
        A[b_jAcol + 7 * j] = A[7 * jp + b_jAcol];
        A[b_jAcol + 7 * jp] = temp;
      }
    }
  }
}

/* Function for MATLAB Function: '<Root>/UKF_correction' */
static void h(const real32_T x[7], const real32_T q[4], real32_T y[6])
{
  real32_T Vair;
  real32_T beta;
  real32_T scale;
  real32_T t;
  real32_T qinter_idx_0;
  real32_T qinv_idx_1;
  real32_T qinv_idx_2;
  real32_T qinv_idx_3;
  real32_T qinter_idx_2;
  real32_T qinter_idx_3;
  real32_T qinter_idx_1;

  /* -----------------Observation_model------------------- */
  /* description : */
  /*  */
  /* Dim of the measurement vector dim(zk) = 10; */
  /* y=[y1 y2 y3 y4 y5 y6 y7 y8 y9 y10] */
  /* y=[V1 V2 V3 P1 P2 P3 B1 B2 B3 YH ] */
  /*  */
  /* Inputs :  */
  /* 		-States:                                x      	[15 x 1] */
  /* 		-Measures :                             zk		[10  x 1] */
  /* Outputs : */
  /* 		-outputs model   :                      y 		[10 x 1] */
  /* ------------------------------------------------------ */
  scale = 1.17549435E-38F;
  beta = fabsf(x[0]);
  if (beta > 1.17549435E-38F) {
    Vair = 1.0F;
    scale = beta;
  } else {
    t = beta / 1.17549435E-38F;
    Vair = t * t;
  }

  beta = fabsf(x[1]);
  if (beta > scale) {
    t = scale / beta;
    Vair = Vair * t * t + 1.0F;
    scale = beta;
  } else {
    t = beta / scale;
    Vair += t * t;
  }

  beta = fabsf(x[2]);
  if (beta > scale) {
    t = scale / beta;
    Vair = Vair * t * t + 1.0F;
    scale = beta;
  } else {
    t = beta / scale;
    Vair += t * t;
  }

  Vair = scale * sqrtf(Vair);

  /* ==> Ground speed|R0 = [u,v,w]|R0 + wind|R0 */
  /* description : */
  /* q*D*q-1  */
  scale = ((q[0] * q[0] + q[1] * q[1]) + q[2] * q[2]) + q[3] * q[3];
  t = q[0] / scale;
  qinv_idx_1 = -q[1] / scale;
  qinv_idx_2 = -q[2] / scale;
  qinv_idx_3 = -q[3] / scale;
  qinter_idx_0 = -((q[1] * x[0] + q[2] * x[1]) + q[3] * x[2]);
  qinter_idx_1 = (q[2] * x[2] - q[3] * x[1]) + x[0] * q[0];
  qinter_idx_2 = (q[3] * x[0] - q[1] * x[2]) + x[1] * q[0];
  qinter_idx_3 = (q[1] * x[1] - q[2] * x[0]) + x[2] * q[0];

  /* ==> Va = scale factor * norm([u,v,w]) */
  if (Vair > 0.0001) {
    /* ==> AOA = atan(w/u) */
    scale = atan2f(x[2], x[0]);

    /* ==> Sideslip = asin(v/||Vair||) */
    beta = asinf(x[1] / Vair);
  } else {
    scale = 0.0F;
    beta = 0.0F;
  }

  y[0] = (((qinter_idx_2 * qinv_idx_3 - qinter_idx_3 * qinv_idx_2) +
           qinter_idx_1 * t) + qinv_idx_1 * qinter_idx_0) + x[3];
  y[1] = (((qinter_idx_3 * qinv_idx_1 - qinter_idx_1 * qinv_idx_3) +
           qinter_idx_2 * t) + qinv_idx_2 * qinter_idx_0) + x[4];
  y[2] = (((qinter_idx_1 * qinv_idx_2 - qinter_idx_2 * qinv_idx_1) +
           qinter_idx_3 * t) + qinv_idx_3 * qinter_idx_0) + x[5];
  y[3] = x[6] * Vair;
  y[4] = scale;
  y[5] = beta;
}

/* Function for MATLAB Function: '<Root>/UKF_prediction' */
static void RungeKutta(const real32_T x[7], real32_T dt, const real32_T u[6],
  real32_T xi[7])
{
  real32_T k2[7];
  real32_T k3[7];
  real32_T k4[7];
  real32_T b_x[7];
  real32_T y;
  int32_T i;

  /* ----------------Runge Kutta 4----------------- */
  /* description : */
  /* dt is an integration step (tk+1-tk) */
  /* h_2 is define as step/2 */
  /*  */
  /* Non linear function f(x,u), results : X(:,k)=[Xk+1(0) ... Xk+1(2n+1)] */
  /* =RungeKutta(X(:,k),dt,omega,a); */
  /* -------------------Process_model--------------------- */
  /* description : */
  /*  */
  /* Dim of the state vector : dim(x) = 6; */
  /* x=[x1 x2 x3 x4 x5 x6] */
  /* x=[uk vk wk uw0 vw0 ww0] */
  /*  */
  /*    dx=f(x,u) */
  /*    dx1=f(x1,u) */
  /*    dx2=f(x2,u) */
  /*    ... */
  /*    dx=[dx1; dx2; ...] */
  /* Inputs :  */
  /* 		-States:                            x      	 */
  /* 		-Command inputs :                   omega,a    */
  /* Outputs : */
  /* 		-state dot   :                      dx 		 */
  /* ------------------------------------------------------ */
  /*  dot_uvw, dot_wind_bias, dot_airspeed_scale */
  for (i = 0; i < 7; i++) {
    xi[i] = 0.0F;
  }

  /* a = single(rep(q, u(4:6))); */
  /* ==> dot_UVW */
  xi[0] = (u[2] * x[1] + u[3]) - u[1] * x[2];
  xi[1] = (u[0] * x[2] + u[4]) - u[2] * x[0];
  xi[2] = (u[1] * x[0] + u[5]) - u[0] * x[1];

  /*   phi = single(atan2(2*(q(1)*q(2)+q(3)*q(4)), 1-2*(q(2)^2+q(3)^2))); */
  /*   theta = single(asin(2*(q(1)*q(3)-q(2)*q(4)))); */
  /*   dx(1)=a(1)-9.81*sin(theta)+ omega(3)* x(2)-omega(2)*x(3); */
  /*   dx(2)=a(2)+9.81*cos(theta)*sin(phi) + omega(1)* x(3)-omega(3)*x(1); */
  /*   dx(3)=a(3)+9.81*cos(theta)*cos(phi) + omega(2)* x(1)-omega(1)*x(2); */
  /* ==> dot_biais */
  /*  already at zero */
  /* -------------------Process_model--------------------- */
  /* description : */
  /*  */
  /* Dim of the state vector : dim(x) = 6; */
  /* x=[x1 x2 x3 x4 x5 x6] */
  /* x=[uk vk wk uw0 vw0 ww0] */
  /*  */
  /*    dx=f(x,u) */
  /*    dx1=f(x1,u) */
  /*    dx2=f(x2,u) */
  /*    ... */
  /*    dx=[dx1; dx2; ...] */
  /* Inputs :  */
  /* 		-States:                            x      	 */
  /* 		-Command inputs :                   omega,a    */
  /* Outputs : */
  /* 		-state dot   :                      dx 		 */
  /* ------------------------------------------------------ */
  /*  dot_uvw, dot_wind_bias, dot_airspeed_scale */
  for (i = 0; i < 7; i++) {
    b_x[i] = xi[i] / 2.0F * dt + x[i];
    k2[i] = 0.0F;
  }

  /* a = single(rep(q, u(4:6))); */
  /* ==> dot_UVW */
  k2[0] = (u[2] * b_x[1] + u[3]) - u[1] * b_x[2];
  k2[1] = (u[0] * b_x[2] + u[4]) - u[2] * b_x[0];
  k2[2] = (u[1] * b_x[0] + u[5]) - u[0] * b_x[1];

  /*   phi = single(atan2(2*(q(1)*q(2)+q(3)*q(4)), 1-2*(q(2)^2+q(3)^2))); */
  /*   theta = single(asin(2*(q(1)*q(3)-q(2)*q(4)))); */
  /*   dx(1)=a(1)-9.81*sin(theta)+ omega(3)* x(2)-omega(2)*x(3); */
  /*   dx(2)=a(2)+9.81*cos(theta)*sin(phi) + omega(1)* x(3)-omega(3)*x(1); */
  /*   dx(3)=a(3)+9.81*cos(theta)*cos(phi) + omega(2)* x(1)-omega(1)*x(2); */
  /* ==> dot_biais */
  /*  already at zero */
  /* -------------------Process_model--------------------- */
  /* description : */
  /*  */
  /* Dim of the state vector : dim(x) = 6; */
  /* x=[x1 x2 x3 x4 x5 x6] */
  /* x=[uk vk wk uw0 vw0 ww0] */
  /*  */
  /*    dx=f(x,u) */
  /*    dx1=f(x1,u) */
  /*    dx2=f(x2,u) */
  /*    ... */
  /*    dx=[dx1; dx2; ...] */
  /* Inputs :  */
  /* 		-States:                            x      	 */
  /* 		-Command inputs :                   omega,a    */
  /* Outputs : */
  /* 		-state dot   :                      dx 		 */
  /* ------------------------------------------------------ */
  /*  dot_uvw, dot_wind_bias, dot_airspeed_scale */
  for (i = 0; i < 7; i++) {
    b_x[i] = k2[i] / 2.0F * dt + x[i];
    k3[i] = 0.0F;
  }

  /* a = single(rep(q, u(4:6))); */
  /* ==> dot_UVW */
  k3[0] = (u[2] * b_x[1] + u[3]) - u[1] * b_x[2];
  k3[1] = (u[0] * b_x[2] + u[4]) - u[2] * b_x[0];
  k3[2] = (u[1] * b_x[0] + u[5]) - u[0] * b_x[1];

  /*   phi = single(atan2(2*(q(1)*q(2)+q(3)*q(4)), 1-2*(q(2)^2+q(3)^2))); */
  /*   theta = single(asin(2*(q(1)*q(3)-q(2)*q(4)))); */
  /*   dx(1)=a(1)-9.81*sin(theta)+ omega(3)* x(2)-omega(2)*x(3); */
  /*   dx(2)=a(2)+9.81*cos(theta)*sin(phi) + omega(1)* x(3)-omega(3)*x(1); */
  /*   dx(3)=a(3)+9.81*cos(theta)*cos(phi) + omega(2)* x(1)-omega(1)*x(2); */
  /* ==> dot_biais */
  /*  already at zero */
  /* -------------------Process_model--------------------- */
  /* description : */
  /*  */
  /* Dim of the state vector : dim(x) = 6; */
  /* x=[x1 x2 x3 x4 x5 x6] */
  /* x=[uk vk wk uw0 vw0 ww0] */
  /*  */
  /*    dx=f(x,u) */
  /*    dx1=f(x1,u) */
  /*    dx2=f(x2,u) */
  /*    ... */
  /*    dx=[dx1; dx2; ...] */
  /* Inputs :  */
  /* 		-States:                            x      	 */
  /* 		-Command inputs :                   omega,a    */
  /* Outputs : */
  /* 		-state dot   :                      dx 		 */
  /* ------------------------------------------------------ */
  /*  dot_uvw, dot_wind_bias, dot_airspeed_scale */
  for (i = 0; i < 7; i++) {
    b_x[i] = dt * k3[i] + x[i];
    k4[i] = 0.0F;
  }

  /* a = single(rep(q, u(4:6))); */
  /* ==> dot_UVW */
  k4[0] = (u[2] * b_x[1] + u[3]) - u[1] * b_x[2];
  k4[1] = (u[0] * b_x[2] + u[4]) - u[2] * b_x[0];
  k4[2] = (u[1] * b_x[0] + u[5]) - u[0] * b_x[1];

  /*   phi = single(atan2(2*(q(1)*q(2)+q(3)*q(4)), 1-2*(q(2)^2+q(3)^2))); */
  /*   theta = single(asin(2*(q(1)*q(3)-q(2)*q(4)))); */
  /*   dx(1)=a(1)-9.81*sin(theta)+ omega(3)* x(2)-omega(2)*x(3); */
  /*   dx(2)=a(2)+9.81*cos(theta)*sin(phi) + omega(1)* x(3)-omega(3)*x(1); */
  /*   dx(3)=a(3)+9.81*cos(theta)*cos(phi) + omega(2)* x(1)-omega(1)*x(2); */
  /* ==> dot_biais */
  /*  already at zero */
  y = dt / 6.0F;
  for (i = 0; i < 7; i++) {
    xi[i] = (((k2[i] + k3[i]) * 2.0F + xi[i]) + k4[i]) * y + x[i];
  }
}

/* Model step function */
void UKF_Wind_Estimator_step(void)
{
  real32_T lambda;
  int32_T jmax;
  int32_T colj;
  int32_T b_j;
  boolean_T exitg1;
  real32_T Y[49];
  real32_T u[6];
  real32_T SQ[49];
  real32_T p[7];
  real32_T unusedU0[147];
  int32_T jj;
  real32_T ajj;
  int32_T ix;
  int32_T iy;
  int32_T b_ix;
  int32_T e;
  int32_T ia;
  real32_T SR[36];
  real32_T unusedU0_0[120];
  real32_T K[42];
  real32_T U[42];
  real32_T d[225];
  real32_T rtb_x1[7];
  real32_T rtb_Wc[15];
  real32_T rtb_Wm[15];
  real32_T rtb_P1[49];
  real32_T rtb_X[105];
  real32_T rtb_Y[105];
  real32_T rtb_Z[90];
  real32_T rtb_z[6];
  int32_T i;
  real32_T lambda_0[120];
  real32_T lambda_1[147];
  int32_T b_iy;
  real32_T lambda_2[49];
  real32_T SR_0[36];
  real32_T lambda_3[36];
  real32_T rtb_Z_0[90];
  real32_T tmp[6];

  /* MATLAB Function: '<Root>/initialization' incorporates:
   *  Inport: '<Root>/P0'
   *  Inport: '<Root>/alpha'
   *  Inport: '<Root>/beta'
   *  Inport: '<Root>/ki'
   *  Inport: '<Root>/x0'
   *  UnitDelay: '<Root>/ Delay'
   *  UnitDelay: '<Root>/ Delay1'
   */
  /* MATLAB Function 'initialization': '<S3>:1' */
  /* ----------------Initialisation--------------------- */
  /* P=0*eye(11); */
  /* x=(linspace(0,0,11))'; */
  /* persistent Q */
  /* persistent R */
  if (!ukf_DW.P_not_empty) {
    /* '<S3>:1:11' */
    /* '<S3>:1:12' */
    memcpy(&SQ[0], &(ukf_init.P0[0]), 49U * sizeof(real32_T));
    jmax = 0;
    colj = 0;
    b_j = 1;
    exitg1 = false;
    while ((!exitg1) && (b_j < 8)) {
      jj = (colj + b_j) - 1;
      lambda = 0.0F;
      if (!(b_j - 1 < 1)) {
        ix = colj;
        iy = colj;
        for (i = 1; i < b_j; i++) {
          lambda += SQ[ix] * SQ[iy];
          ix++;
          iy++;
        }
      }

      ajj = SQ[jj] - lambda;
      if (ajj > 0.0F) {
        ajj = sqrtf(ajj);
        SQ[jj] = ajj;
        if (b_j < 7) {
          if (b_j - 1 != 0) {
            b_iy = jj + 7;
            ix = ((6 - b_j) * 7 + colj) + 8;
            for (iy = colj + 8; iy <= ix; iy += 7) {
              b_ix = colj;
              lambda = 0.0F;
              e = (iy + b_j) - 2;
              for (ia = iy; ia <= e; ia++) {
                lambda += SQ[ia - 1] * SQ[b_ix];
                b_ix++;
              }

              SQ[b_iy] += -lambda;
              b_iy += 7;
            }
          }

          lambda = 1.0F / ajj;
          ix = ((6 - b_j) * 7 + jj) + 8;
          for (b_iy = jj + 7; b_iy + 1 <= ix; b_iy += 7) {
            SQ[b_iy] *= lambda;
          }

          colj += 7;
        }

        b_j++;
      } else {
        SQ[jj] = ajj;
        jmax = b_j;
        exitg1 = true;
      }
    }

    if (jmax == 0) {
      jmax = 7;
    } else {
      jmax--;
    }

    for (b_j = 0; b_j + 1 <= jmax; b_j++) {
      for (colj = b_j + 1; colj + 1 <= jmax; colj++) {
        SQ[colj + 7 * b_j] = 0.0F;
      }
    }

    for (jmax = 0; jmax < 7; jmax++) {
      for (b_j = 0; b_j < 7; b_j++) {
        rtb_P1[b_j + 7 * jmax] = SQ[7 * b_j + jmax];
      }
    }

    ukf_DW.P_not_empty = true;
  } else {
    /* '<S3>:1:14' */
    memcpy(&rtb_P1[0], &ukf_DW.Delay_DSTATE[0], 49U * sizeof(real32_T));
  }

  if (!ukf_DW.x_not_empty) {
    /* '<S3>:1:17' */
    /* '<S3>:1:18' */
    for (i = 0; i < 7; i++) {
      rtb_x1[i] = ukf_init.x0[i];
    }

    ukf_DW.x_not_empty = true;
  } else {
    /* '<S3>:1:20' */
    for (i = 0; i < 7; i++) {
      rtb_x1[i] = ukf_DW.Delay1_DSTATE[i];
    }
  }

  /* %%%%%%%%%%%%%%%--PROGRAMME PRINCIPAL--%%%%%%%%%%%%%%%% */
  /* alpha=0.01;  %10^-1=<alpha=<1 : dispersion des sigma points. */
  /* Default alpha=0.5; */
  /* ki=0;       %ki>=0 : garantis que la covariance soit semi-definit positive. */
  /* Default k=0; */
  /* beta=2;     %informations sur les moments d'ordre 3 */
  /* Default beta=2 choix optimal quand la distrib est sup. gaussienne. */
  /* dt=0.02;    %dt pour le Runge-Kutta */
  /* --------Calcul du lambda----------------- */
  /* '<S3>:1:40' */
  /* dimension de l'etat */
  /* '<S3>:1:42' */
  lambda = ukf_init.alpha * ukf_init.alpha * (7.0F + ukf_init.ki) - 7.0F;

  /* '<S3>:1:43' */
  /* ----------------Calcul des ponderations Wm et Wc-------------------- */
  /* '<S3>:1:47' */
  ajj = 0.5F / (7.0F + lambda);
  rtb_Wm[0] = lambda / (7.0F + lambda);
  for (i = 0; i < 14; i++) {
    rtb_Wm[i + 1] = ajj;
  }

  /* '<S3>:1:48' */
  for (i = 0; i < 15; i++) {
    rtb_Wc[i] = rtb_Wm[i];
  }

  /* '<S3>:1:49' */
  rtb_Wc[0] = ((1.0F - ukf_init.alpha * ukf_init.alpha) + ukf_init.beta) +
    rtb_Wm[0];

  /* '<S3>:1:50' */
  /* '<S3>:1:51' */
  /* '<S3>:1:52' */
  lambda = sqrtf(7.0F + lambda);

  /* MATLAB Function: '<Root>/sigmas' */
  /* MATLAB Function 'sigmas': '<S5>:1' */
  /* ----------------Sigma-point--------------------------- */
  /* sigmas points et concatenation : n*(2n+1) */
  /* Inputs :  */
  /* 		-States:                                 x 		 */
  /* 		-Covariance on state :                   P 		 */
  /* 		-Weighting squate-root  */
  /* 		-sigmas-point      (Wm,Wc)               c		 */
  /* Outputs :   */
  /* 		-Sigmas-points (state) :                 X      	 */
  /* ---------------------------------------------------- */
  /* Covariance with ponderation: A = c * P  */
  /* '<S5>:1:15' */
  for (jmax = 0; jmax < 49; jmax++) {
    SQ[jmax] = lambda * rtb_P1[jmax];
  }

  /* dimension modification on state  */
  /* '<S5>:1:18' */
  /* concatenation : X=[x Y+A Y-A] */
  /* '<S5>:1:21' */
  for (jmax = 0; jmax < 7; jmax++) {
    for (b_j = 0; b_j < 7; b_j++) {
      Y[b_j + 7 * jmax] = rtb_x1[b_j];
    }

    rtb_X[jmax] = rtb_x1[jmax];
  }

  for (jmax = 0; jmax < 49; jmax++) {
    rtb_X[jmax + 7] = Y[jmax] + SQ[jmax];
    rtb_X[jmax + 56] = Y[jmax] - SQ[jmax];
  }

  /* End of MATLAB Function: '<Root>/sigmas' */

  /* MATLAB Function: '<Root>/UKF_prediction' incorporates:
   *  Inport: '<Root>/Q'
   *  Inport: '<Root>/accel'
   *  Inport: '<Root>/dt'
   *  Inport: '<Root>/rates'
   *  MATLAB Function: '<Root>/initialization'
   */
  /* MATLAB Function 'UKF_prediction': '<S2>:1' */
  /* '<S2>:1:38' */
  /* '<S2>:1:31' */
  /* ----------------Prediction Step------------------------- */
  /* Sigma point and model fonction f(x,u) */
  /* Inputs :  */
  /* 		-Sigmas-points (state) :                X      	 */
  /* 		-Weighting sigmas-points  :             Wm, Wc 	 */
  /* 		-Command input	 :                      omega, a    */
  /* 		-Dimension vector state:                n		 */
  /* 		-Integration step :                     dt		 */
  /* 		-Dimension 2n+1 :                       L  		 */
  /* 		-Weighting state (Wk) :                 Q  		 */
  /* Outputs : */
  /* 		-Predicted state:                       y 		 */
  /* 		-Sigmas_state   :                       Y 		 */
  /* 		-Covariance on state :                  H */
  /* ---------------------------------------------------- */
  /* ----------------Prediction------------------------- */
  /* description : */
  /* Output : SigmaX(k+1) et xmoy(k+1) */
  /*  */
  /*  input state size */
  /*  sigma state size */
  /* '<S2>:1:26' */
  for (i = 0; i < 7; i++) {
    rtb_x1[i] = 0.0F;
  }

  /* '<S2>:1:27' */
  memset(&rtb_Y[0], 0, 105U * sizeof(real32_T));

  /* '<S2>:1:29' */
  u[0] = ukf_U.rates[0];
  u[3] = ukf_U.accel[0];
  u[1] = ukf_U.rates[1];
  u[4] = ukf_U.accel[1];
  u[2] = ukf_U.rates[2];
  u[5] = ukf_U.accel[2];

  /* '<S2>:1:30' */
  for (i = 0; i < 15; i++) {
    /* '<S2>:1:30' */
    /* '<S2>:1:31' */
    RungeKutta(&rtb_X[7 * i], ukf_params.dt, u, &rtb_Y[7 * i]);

    /* non linear function */
    /* '<S2>:1:32' */
    for (jmax = 0; jmax < 7; jmax++) {
      rtb_x1[jmax] += rtb_Y[7 * i + jmax] * rtb_Wm[i];
    }

    /* predicted vector on the state */
    /* y = Y(:,1); */
    /* '<S2>:1:30' */
  }

  /* '<S2>:1:36' */
  for (b_iy = 0; b_iy < 49; b_iy++) {
    SQ[b_iy] = sqrtf(ukf_params.Q[b_iy]);
  }

  /* square-root of R */
  /* '<S2>:1:38' */
  lambda = sqrtf(rtb_Wm[1]);

  /* '<S2>:1:40' */
  /* 33*11=>[11*22 11*11] */
  /* '<S2>:1:42' */
  for (jmax = 0; jmax < 7; jmax++) {
    for (b_j = 0; b_j < 14; b_j++) {
      lambda_1[b_j + 21 * jmax] = (rtb_Y[(1 + b_j) * 7 + jmax] - rtb_x1[jmax]) *
        lambda;
    }
  }

  for (jmax = 0; jmax < 7; jmax++) {
    for (b_j = 0; b_j < 7; b_j++) {
      lambda_1[(b_j + 21 * jmax) + 14] = SQ[7 * b_j + jmax];
    }
  }

  qr_e(lambda_1, unusedU0, SQ);

  /* '<S2>:1:44' */
  for (jmax = 0; jmax < 7; jmax++) {
    p[jmax] = rtb_Y[jmax] - rtb_x1[jmax];
  }

  if (rtb_Wc[0] < 0.0F) {
    /* '<S2>:1:46' */
    /* '<S2>:1:47' */
    lambda = powf(-rtb_Wc[0], 0.25F);
    ajj = powf(-rtb_Wc[0], 0.25F);
    for (jmax = 0; jmax < 7; jmax++) {
      for (b_j = 0; b_j < 7; b_j++) {
        Y[jmax + 7 * b_j] = 0.0F;
        for (colj = 0; colj < 7; colj++) {
          Y[jmax + 7 * b_j] += SQ[7 * jmax + colj] * SQ[7 * b_j + colj];
        }

        lambda_2[jmax + 7 * b_j] = lambda * p[jmax] * (ajj * p[b_j]);
      }
    }

    for (jmax = 0; jmax < 7; jmax++) {
      for (b_j = 0; b_j < 7; b_j++) {
        rtb_P1[b_j + 7 * jmax] = Y[7 * jmax + b_j] - lambda_2[7 * jmax + b_j];
      }
    }

    b_j = 0;
    i = 0;
    jmax = 1;
    exitg1 = false;
    while ((!exitg1) && (jmax < 8)) {
      colj = (i + jmax) - 1;
      lambda = 0.0F;
      if (!(jmax - 1 < 1)) {
        jj = i;
        b_iy = i;
        for (ix = 1; ix < jmax; ix++) {
          lambda += rtb_P1[jj] * rtb_P1[b_iy];
          jj++;
          b_iy++;
        }
      }

      lambda = rtb_P1[colj] - lambda;
      if (lambda > 0.0F) {
        lambda = sqrtf(lambda);
        rtb_P1[colj] = lambda;
        if (jmax < 7) {
          if (jmax - 1 != 0) {
            jj = colj + 7;
            b_iy = ((6 - jmax) * 7 + i) + 8;
            for (ix = i + 8; ix <= b_iy; ix += 7) {
              iy = i;
              ajj = 0.0F;
              b_ix = (ix + jmax) - 2;
              for (e = ix; e <= b_ix; e++) {
                ajj += rtb_P1[e - 1] * rtb_P1[iy];
                iy++;
              }

              rtb_P1[jj] += -ajj;
              jj += 7;
            }
          }

          lambda = 1.0F / lambda;
          jj = ((6 - jmax) * 7 + colj) + 8;
          for (colj += 7; colj + 1 <= jj; colj += 7) {
            rtb_P1[colj] *= lambda;
          }

          i += 7;
        }

        jmax++;
      } else {
        rtb_P1[colj] = lambda;
        b_j = jmax;
        exitg1 = true;
      }
    }

    if (b_j == 0) {
      i = 7;
    } else {
      i = b_j - 1;
    }

    for (b_j = 0; b_j + 1 <= i; b_j++) {
      for (jmax = b_j + 1; jmax + 1 <= i; jmax++) {
        rtb_P1[jmax + 7 * b_j] = 0.0F;
      }
    }
  } else {
    /* '<S2>:1:49' */
    lambda = powf(rtb_Wc[0], 0.25F);
    ajj = powf(rtb_Wc[0], 0.25F);
    for (jmax = 0; jmax < 7; jmax++) {
      for (b_j = 0; b_j < 7; b_j++) {
        Y[jmax + 7 * b_j] = 0.0F;
        for (colj = 0; colj < 7; colj++) {
          Y[jmax + 7 * b_j] += SQ[7 * jmax + colj] * SQ[7 * b_j + colj];
        }

        lambda_2[jmax + 7 * b_j] = lambda * p[jmax] * (ajj * p[b_j]);
      }
    }

    for (jmax = 0; jmax < 7; jmax++) {
      for (b_j = 0; b_j < 7; b_j++) {
        rtb_P1[b_j + 7 * jmax] = Y[7 * jmax + b_j] + lambda_2[7 * jmax + b_j];
      }
    }

    jmax = 0;
    colj = 0;
    b_j = 1;
    exitg1 = false;
    while ((!exitg1) && (b_j < 8)) {
      jj = (colj + b_j) - 1;
      lambda = 0.0F;
      if (!(b_j - 1 < 1)) {
        ix = colj;
        iy = colj;
        for (b_iy = 1; b_iy < b_j; b_iy++) {
          lambda += rtb_P1[ix] * rtb_P1[iy];
          ix++;
          iy++;
        }
      }

      ajj = rtb_P1[jj] - lambda;
      if (ajj > 0.0F) {
        ajj = sqrtf(ajj);
        rtb_P1[jj] = ajj;
        if (b_j < 7) {
          if (b_j - 1 != 0) {
            b_iy = jj + 7;
            ix = ((6 - b_j) * 7 + colj) + 8;
            for (iy = colj + 8; iy <= ix; iy += 7) {
              b_ix = colj;
              lambda = 0.0F;
              e = (iy + b_j) - 2;
              for (ia = iy; ia <= e; ia++) {
                lambda += rtb_P1[ia - 1] * rtb_P1[b_ix];
                b_ix++;
              }

              rtb_P1[b_iy] += -lambda;
              b_iy += 7;
            }
          }

          lambda = 1.0F / ajj;
          ix = ((6 - b_j) * 7 + jj) + 8;
          for (i = jj + 7; i + 1 <= ix; i += 7) {
            rtb_P1[i] *= lambda;
          }

          colj += 7;
        }

        b_j++;
      } else {
        rtb_P1[jj] = ajj;
        jmax = b_j;
        exitg1 = true;
      }
    }

    if (jmax == 0) {
      jmax = 7;
    } else {
      jmax--;
    }

    for (b_j = 0; b_j + 1 <= jmax; b_j++) {
      for (colj = b_j + 1; colj + 1 <= jmax; colj++) {
        rtb_P1[colj + 7 * b_j] = 0.0F;
      }
    }
  }

  /* End of MATLAB Function: '<Root>/UKF_prediction' */

  /* MATLAB Function: '<Root>/UKF_correction' incorporates:
   *  Inport: '<Root>/R'
   *  Inport: '<Root>/q'
   *  MATLAB Function: '<Root>/initialization'
   */
  /* MATLAB Function 'UKF_correction': '<S1>:1' */
  /* '<S1>:1:49' */
  /* '<S1>:1:41' */
  /* '<S1>:1:38' */
  /* ----------------Correction Step------------------------- */
  /* %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */
  /*       By Condomines Jean_Philippe - 1 may 2015-    */
  /*                    jp.condomines@gmail.com */
  /* %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */
  /* Sigma point and output fonction  h(x_k) */
  /* Inputs :  */
  /* 		-Sigmas-points (state)   :                  X      	 */
  /* 		-Weighting sigmas-points  :                 Wm, Wc 	 */
  /* 		-Dimension of the measurement vector:       n		 */
  /* 		-Dimension 2n+1 :                           L  		 */
  /* 		-Weighting measures (Vk+1):                 R  		 */
  /* 		-Measures :                                 zk 		 */
  /* Outputs : */
  /* 		-Sigmas_measures :                          Z 		 */
  /* 		-Predicted measurement :                    z 		 */
  /* 		-Covariance on measures :                   J 		 */
  /* ---------------------------------------------------- */
  /*  measurement input size */
  /*  input state size */
  /*  sigma state size */
  /* Zeros vector for predicted measures */
  /* '<S1>:1:29' */
  for (i = 0; i < 6; i++) {
    rtb_z[i] = 0.0F;
  }

  /* Zeros vector for predicted Sigmas_measures */
  /* '<S1>:1:32' */
  /* "for" loop for "measures predicted" and "sigmas_measures" predicted */
  /* '<S1>:1:35' */
  for (i = 0; i < 15; i++) {
    /* '<S1>:1:35' */
    /* non linear function h(x), results : Z(:,k)=[zk+1(0) ... zk+1(2n+1)] */
    /* '<S1>:1:38' */
    h(&rtb_Y[7 * i], ukf_U.q, u);
    for (jmax = 0; jmax < 6; jmax++) {
      rtb_Z[jmax + 6 * i] = u[jmax];
    }

    /* Predicted measurement */
    /* '<S1>:1:41' */
    for (jmax = 0; jmax < 6; jmax++) {
      rtb_z[jmax] += rtb_Z[6 * i + jmax] * rtb_Wm[i];
    }

    /*   z = Z(:,1); */
    /* '<S1>:1:35' */
  }

  /* Square-root of R */
  /* '<S1>:1:46' */
  for (b_iy = 0; b_iy < 36; b_iy++) {
    SR[b_iy] = sqrtf(ukf_params.R[b_iy]);
  }

  /* Subtracting of "predicted Sigma measures" et "predicted measurement" with ponderation "Wc" */
  /* '<S1>:1:49' */
  lambda = sqrtf(rtb_Wm[1]);

  /* Concatenation with the square root of the matrix "R" */
  /* '<S1>:1:52' */
  /* factorization qr, result use in "J" and rest in "Temp"  */
  /* Sz,k+1 = qr{[sqrt(Wc(1)) * [Zk+1(:,2:2n+1)-zk+1]  sqrt(R)] */
  /* '<S1>:1:56' */
  for (jmax = 0; jmax < 6; jmax++) {
    for (b_j = 0; b_j < 14; b_j++) {
      lambda_0[b_j + 20 * jmax] = (rtb_Z[(1 + b_j) * 6 + jmax] - rtb_z[jmax]) *
        lambda;
    }
  }

  for (jmax = 0; jmax < 6; jmax++) {
    for (b_j = 0; b_j < 6; b_j++) {
      lambda_0[(b_j + 20 * jmax) + 14] = SR[6 * b_j + jmax];
    }
  }

  qr(lambda_0, unusedU0_0, SR);

  /* Subtracting the first column "zk+1(0)" with all the measurements zk+1 predicted */
  /* '<S1>:1:59' */
  for (jmax = 0; jmax < 6; jmax++) {
    u[jmax] = rtb_Z[jmax] - rtb_z[jmax];
  }

  /* Test on Wc(1) for cholesky factorization : Cholupdate{Sz,k+1,p,Wc(0)}  */
  if (rtb_Wc[0] < 0.0F) {
    /* '<S1>:1:62' */
    /* '<S1>:1:63' */
    lambda = powf(-rtb_Wc[0], 0.25F);
    ajj = powf(-rtb_Wc[0], 0.25F);
    for (jmax = 0; jmax < 6; jmax++) {
      for (b_j = 0; b_j < 6; b_j++) {
        SR_0[jmax + 6 * b_j] = 0.0F;
        for (colj = 0; colj < 6; colj++) {
          SR_0[jmax + 6 * b_j] += SR[6 * jmax + colj] * SR[6 * b_j + colj];
        }

        lambda_3[jmax + 6 * b_j] = lambda * u[jmax] * (ajj * u[b_j]);
      }
    }

    for (jmax = 0; jmax < 6; jmax++) {
      for (b_j = 0; b_j < 6; b_j++) {
        SR[b_j + 6 * jmax] = SR_0[6 * jmax + b_j] - lambda_3[6 * jmax + b_j];
      }
    }

    b_j = 0;
    i = 0;
    jmax = 1;
    exitg1 = false;
    while ((!exitg1) && (jmax < 7)) {
      colj = (i + jmax) - 1;
      lambda = 0.0F;
      if (!(jmax - 1 < 1)) {
        jj = i;
        b_iy = i;
        for (ix = 1; ix < jmax; ix++) {
          lambda += SR[jj] * SR[b_iy];
          jj++;
          b_iy++;
        }
      }

      lambda = SR[colj] - lambda;
      if (lambda > 0.0F) {
        lambda = sqrtf(lambda);
        SR[colj] = lambda;
        if (jmax < 6) {
          if (jmax - 1 != 0) {
            jj = colj + 6;
            b_iy = ((5 - jmax) * 6 + i) + 7;
            for (ix = i + 7; ix <= b_iy; ix += 6) {
              iy = i;
              ajj = 0.0F;
              b_ix = (ix + jmax) - 2;
              for (e = ix; e <= b_ix; e++) {
                ajj += SR[e - 1] * SR[iy];
                iy++;
              }

              SR[jj] += -ajj;
              jj += 6;
            }
          }

          lambda = 1.0F / lambda;
          jj = ((5 - jmax) * 6 + colj) + 7;
          for (colj += 6; colj + 1 <= jj; colj += 6) {
            SR[colj] *= lambda;
          }

          i += 6;
        }

        jmax++;
      } else {
        SR[colj] = lambda;
        b_j = jmax;
        exitg1 = true;
      }
    }

    if (b_j == 0) {
      i = 6;
    } else {
      i = b_j - 1;
    }

    for (b_j = 0; b_j + 1 <= i; b_j++) {
      for (jmax = b_j + 1; jmax + 1 <= i; jmax++) {
        SR[jmax + 6 * b_j] = 0.0F;
      }
    }
  } else {
    /* '<S1>:1:65' */
    lambda = powf(rtb_Wc[0], 0.25F);
    ajj = powf(rtb_Wc[0], 0.25F);
    for (jmax = 0; jmax < 6; jmax++) {
      for (b_j = 0; b_j < 6; b_j++) {
        SR_0[jmax + 6 * b_j] = 0.0F;
        for (colj = 0; colj < 6; colj++) {
          SR_0[jmax + 6 * b_j] += SR[6 * jmax + colj] * SR[6 * b_j + colj];
        }

        lambda_3[jmax + 6 * b_j] = lambda * u[jmax] * (ajj * u[b_j]);
      }
    }

    for (jmax = 0; jmax < 6; jmax++) {
      for (b_j = 0; b_j < 6; b_j++) {
        SR[b_j + 6 * jmax] = SR_0[6 * jmax + b_j] + lambda_3[6 * jmax + b_j];
      }
    }

    jmax = 0;
    colj = 0;
    b_j = 1;
    exitg1 = false;
    while ((!exitg1) && (b_j < 7)) {
      jj = (colj + b_j) - 1;
      lambda = 0.0F;
      if (!(b_j - 1 < 1)) {
        ix = colj;
        iy = colj;
        for (b_iy = 1; b_iy < b_j; b_iy++) {
          lambda += SR[ix] * SR[iy];
          ix++;
          iy++;
        }
      }

      ajj = SR[jj] - lambda;
      if (ajj > 0.0F) {
        ajj = sqrtf(ajj);
        SR[jj] = ajj;
        if (b_j < 6) {
          if (b_j - 1 != 0) {
            b_iy = jj + 6;
            ix = ((5 - b_j) * 6 + colj) + 7;
            for (iy = colj + 7; iy <= ix; iy += 6) {
              b_ix = colj;
              lambda = 0.0F;
              e = (iy + b_j) - 2;
              for (ia = iy; ia <= e; ia++) {
                lambda += SR[ia - 1] * SR[b_ix];
                b_ix++;
              }

              SR[b_iy] += -lambda;
              b_iy += 6;
            }
          }

          lambda = 1.0F / ajj;
          ix = ((5 - b_j) * 6 + jj) + 7;
          for (i = jj + 6; i + 1 <= ix; i += 6) {
            SR[i] *= lambda;
          }

          colj += 6;
        }

        b_j++;
      } else {
        SR[jj] = ajj;
        jmax = b_j;
        exitg1 = true;
      }
    }

    if (jmax == 0) {
      jmax = 6;
    } else {
      jmax--;
    }

    for (b_j = 0; b_j + 1 <= jmax; b_j++) {
      for (colj = b_j + 1; colj + 1 <= jmax; colj++) {
        SR[colj + 6 * b_j] = 0.0F;
      }
    }
  }

  /* Transposition to have a lower triangular matrix */
  /* '<S1>:1:69' */
  for (jmax = 0; jmax < 6; jmax++) {
    for (b_j = 0; b_j < 6; b_j++) {
      SR_0[b_j + 6 * jmax] = SR[6 * b_j + jmax];
    }
  }

  for (jmax = 0; jmax < 6; jmax++) {
    for (b_j = 0; b_j < 6; b_j++) {
      SR[b_j + 6 * jmax] = SR_0[6 * jmax + b_j];
    }
  }

  /* End of MATLAB Function: '<Root>/UKF_correction' */

  /* MATLAB Function: '<Root>/main' incorporates:
   *  Inport: '<Root>/aoa'
   *  Inport: '<Root>/sideslip'
   *  Inport: '<Root>/va '
   *  Inport: '<Root>/vk'
   */
  memcpy(&Y[0], &rtb_P1[0], 49U * sizeof(real32_T));

  /* MATLAB Function 'main': '<S4>:1' */
  /* '<S4>:1:9' */
  /*  measurement input size */
  /*  input state size */
  /*  sigma state size */
  /* '<S4>:1:7' */
  /* '<S4>:1:9' */
  memset(&d[0], 0, 225U * sizeof(real32_T));

  /* Kalman gain : (Pxz/Sz')/Sz */
  /* Caution "/" is least squares root  for "Ax = b" */
  /* '<S4>:1:13' */
  for (b_j = 0; b_j < 15; b_j++) {
    d[b_j + 15 * b_j] = rtb_Wc[b_j];
    for (jmax = 0; jmax < 7; jmax++) {
      rtb_X[jmax + 7 * b_j] = rtb_Y[7 * b_j + jmax] - rtb_x1[jmax];
    }
  }

  for (jmax = 0; jmax < 7; jmax++) {
    for (b_j = 0; b_j < 15; b_j++) {
      rtb_Y[jmax + 7 * b_j] = 0.0F;
      for (colj = 0; colj < 15; colj++) {
        rtb_Y[jmax + 7 * b_j] += rtb_X[7 * colj + jmax] * d[15 * b_j + colj];
      }
    }
  }

  for (jmax = 0; jmax < 6; jmax++) {
    for (b_j = 0; b_j < 15; b_j++) {
      rtb_Z_0[b_j + 15 * jmax] = rtb_Z[6 * b_j + jmax] - rtb_z[jmax];
    }
  }

  for (jmax = 0; jmax < 7; jmax++) {
    for (b_j = 0; b_j < 6; b_j++) {
      K[jmax + 7 * b_j] = 0.0F;
      for (colj = 0; colj < 15; colj++) {
        K[jmax + 7 * b_j] += rtb_Y[7 * colj + jmax] * rtb_Z_0[15 * b_j + colj];
      }
    }
  }

  for (jmax = 0; jmax < 6; jmax++) {
    for (b_j = 0; b_j < 6; b_j++) {
      SR_0[b_j + 6 * jmax] = SR[6 * b_j + jmax];
    }
  }

  mrdivide(K, SR_0);
  mrdivide(K, SR);

  /* Predicted state with Kalman gain "K" : x = xpred + K * (zk - z) */
  /* '<S4>:1:16' */
  /* Quaternion normalisation */
  /* x(1:4)=normalQ(x(1:4)'); */
  /* Cholesky factorization : U = K * Sz    */
  /* '<S4>:1:22' */
  for (jmax = 0; jmax < 7; jmax++) {
    for (b_j = 0; b_j < 6; b_j++) {
      U[jmax + 7 * b_j] = 0.0F;
      for (colj = 0; colj < 6; colj++) {
        U[jmax + 7 * b_j] += K[7 * colj + jmax] * SR[6 * b_j + colj];
      }
    }
  }

  /* "for" loop for cholesky factorization */
  /* '<S4>:1:25' */
  /* "Sk" in "P" and Transpose of "P" */
  /* '<S4>:1:30' */
  /* SqrtP = sqrt(diag(Sk*Sk')); */
  tmp[0] = ukf_U.vk[0];
  tmp[1] = ukf_U.vk[1];
  tmp[2] = ukf_U.vk[2];
  tmp[3] = ukf_U.va;
  tmp[4] = ukf_U.aoa;
  tmp[5] = ukf_U.sideslip;
  for (i = 0; i < 6; i++) {
    /* '<S4>:1:25' */
    /* '<S4>:1:26' */
    for (jmax = 0; jmax < 7; jmax++) {
      for (b_j = 0; b_j < 7; b_j++) {
        rtb_P1[jmax + 7 * b_j] = 0.0F;
        for (colj = 0; colj < 7; colj++) {
          rtb_P1[jmax + 7 * b_j] += Y[7 * jmax + colj] * Y[7 * b_j + colj];
        }

        lambda_2[jmax + 7 * b_j] = U[7 * i + jmax] * U[7 * i + b_j];
      }
    }

    for (jmax = 0; jmax < 7; jmax++) {
      for (b_j = 0; b_j < 7; b_j++) {
        SQ[b_j + 7 * jmax] = rtb_P1[7 * jmax + b_j] - lambda_2[7 * jmax + b_j];
      }
    }

    jmax = 0;
    colj = 0;
    b_j = 1;
    exitg1 = false;
    while ((!exitg1) && (b_j < 8)) {
      jj = (colj + b_j) - 1;
      lambda = 0.0F;
      if (!(b_j - 1 < 1)) {
        ix = colj;
        iy = colj;
        for (b_iy = 1; b_iy < b_j; b_iy++) {
          lambda += SQ[ix] * SQ[iy];
          ix++;
          iy++;
        }
      }

      ajj = SQ[jj] - lambda;
      if (ajj > 0.0F) {
        ajj = sqrtf(ajj);
        SQ[jj] = ajj;
        if (b_j < 7) {
          if (b_j - 1 != 0) {
            b_iy = jj + 7;
            ix = ((6 - b_j) * 7 + colj) + 8;
            for (iy = colj + 8; iy <= ix; iy += 7) {
              b_ix = colj;
              lambda = 0.0F;
              e = (iy + b_j) - 2;
              for (ia = iy; ia <= e; ia++) {
                lambda += SQ[ia - 1] * SQ[b_ix];
                b_ix++;
              }

              SQ[b_iy] += -lambda;
              b_iy += 7;
            }
          }

          lambda = 1.0F / ajj;
          ix = ((6 - b_j) * 7 + jj) + 8;
          for (b_iy = jj + 7; b_iy + 1 <= ix; b_iy += 7) {
            SQ[b_iy] *= lambda;
          }

          colj += 7;
        }

        b_j++;
      } else {
        SQ[jj] = ajj;
        jmax = b_j;
        exitg1 = true;
      }
    }

    if (jmax == 0) {
      jmax = 7;
    } else {
      jmax--;
    }

    for (b_j = 0; b_j + 1 <= jmax; b_j++) {
      for (colj = b_j + 1; colj + 1 <= jmax; colj++) {
        SQ[colj + 7 * b_j] = 0.0F;
      }
    }

    memcpy(&Y[0], &SQ[0], 49U * sizeof(real32_T));

    /* '<S4>:1:25' */
    u[i] = tmp[i] - rtb_z[i];
  }

  for (i = 0; i < 7; i++) {
    lambda = 0.0F;
    for (jmax = 0; jmax < 6; jmax++) {
      lambda += K[7 * jmax + i] * u[jmax];
    }

    p[i] = rtb_x1[i] + lambda;
    for (jmax = 0; jmax < 7; jmax++) {
      rtb_P1[jmax + 7 * i] = Y[7 * jmax + i];
    }

    /* Outport: '<Root>/xout' */
    ukf_Y.xout[i] = p[i];
  }

  /* End of MATLAB Function: '<Root>/main' */

  /* Outport: '<Root>/Pout' */
  memcpy(&ukf_Y.Pout[0], &rtb_P1[0], 49U * sizeof(real32_T));

  /* Update for UnitDelay: '<Root>/ Delay1' */
  for (i = 0; i < 7; i++) {
    ukf_DW.Delay1_DSTATE[i] = p[i];
  }

  /* End of Update for UnitDelay: '<Root>/ Delay1' */

  /* Update for UnitDelay: '<Root>/ Delay' */
  memcpy(&ukf_DW.Delay_DSTATE[0], &rtb_P1[0], 49U * sizeof(real32_T));
}

/* Model initialize function */
void UKF_Wind_Estimator_initialize(void)
{
  /* (no initialization code required) */
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
