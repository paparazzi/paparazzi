/*
 * Copyright (C) 2014 Hann Woei Ho
 *          Guido de Croon
 *
 *
 * This file is part of Paparazzi.
 *
 * Paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * Paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 */

/*
 * @file modules/computer_vision/opticflow/linear_flow_fit.c
 * @brief Takes a set of optical flow vectors and extracts information such as relative velocities and surface roughness.
 *
 * A horizontal and vertical linear fit is made with the optical flow vectors, and from the fit parameters information is extracted such as relative velocities (useful for time-to-contact determination), slope angle, and surface roughness.
 *
 * Code based on the article:
 * "Optic-flow based slope estimation for autonomous landing",
 * de Croon, G.C.H.E., and Ho, H.W., and De Wagter, C., and van Kampen, E., and Remes B., and Chu, Q.P.,
 * in the International Journal of Micro Air Vehicles, Volume 5, Number 4, pages 287 – 297, (2013)
 */

#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <string.h>
//#include "defs_and_types.h"
#include "linear_flow_fit.h"
#include "math/pprz_algebra_float.h"
#include "math/pprz_matrix_decomp_float.h"
#include "math/pprz_simple_matrix.h"

// Is this still necessary?
#define MAX_COUNT_PT 50

#define MIN_SAMPLES_FIT 3
#define NO_FIT 0
#define FIT 1

/**
 * Analyze a linear flow field, retrieving information such as divergence, surface roughness, focus of expansion, etc.
 * @param[out] outcome If 0, there were too few vectors for a fit. If 1, the fit was successful.
 * @param[in] vectors The optical flow vectors
 * @param[in] count The number of optical flow vectors
 * @param[in] error_threshold Error used to determine inliers / outliers.
 * @param[in] n_iterations Number of RANSAC iterations.
 * @param[in] n_samples Number of samples used for a single fit (min. 3).
 * @param[in] im_width Image width in pixels
 * @param[in] im_height Image height in pixels
 * @param[out] info Contains all info extracted from the linear flow fit.
 */
int analyze_linear_flow_field(struct flow_t *vectors, int count, float error_threshold, int n_iterations, int n_samples, int im_width, int im_height, struct linear_flow_fit_info *info)
{
  // Are there enough flow vectors to perform a fit?
  if (count < MIN_SAMPLES_FIT) {
    // return that no fit was made:
    return NO_FIT;
  }

  // fit linear flow field:
  float parameters_u[3], parameters_v[3], min_error_u, min_error_v;
  fit_linear_flow_field(vectors, count, n_iterations, error_threshold, n_samples, parameters_u, parameters_v, &info->fit_error, &min_error_u, &min_error_v, &info->n_inliers_u, &info->n_inliers_v);

  // extract information from the parameters:
  extract_information_from_parameters(parameters_u, parameters_v, im_width, im_height, info);

  // surface roughness is equal to fit error:
  info->surface_roughness = info->fit_error;
  info->divergence = info->relative_velocity_z;
  float diverg = (abs(info->divergence) < 1E-5) ? 1E-5 : info->divergence;
  info->time_to_contact = 1.0f / diverg;

  // return successful fit:
  return FIT;
}

/**
 * Analyze a linear flow field, retrieving information such as divergence, surface roughness, focus of expansion, etc.
 * @param[in] vectors The optical flow vectors
 * @param[in] count The number of optical flow vectors
 * @param[in] error_threshold Error used to determine inliers / outliers.
 * @param[in] n_iterations Number of RANSAC iterations.
 * @param[in] n_samples Number of samples used for a single fit (min. 3).
 * @param[out] parameters_u* Parameters of the horizontal flow field
 * @param[out] parameters_v* Parameters of the vertical flow field
 * @param[out] fit_error* Total error of the finally selected fit
 * @param[out] min_error_u* Error fit horizontal flow field
 * @param[out] min_error_v* Error fit vertical flow field
 * @param[out] n_inliers_u* Number of inliers in the horizontal flow fit.
 * @param[out] n_inliers_v* Number of inliers in the vertical flow fit.
 */
void fit_linear_flow_field(struct flow_t *vectors, int count, float error_threshold, int n_iterations, int n_samples, float *parameters_u, float *parameters_v, float *fit_error, float *min_error_u, float *min_error_v, int *n_inliers_u, int *n_inliers_v)
{

  // We will solve systems of the form A x = b,
  // where A = [nx3] matrix with entries [x, y, 1] for each optic flow location
  // and b = [nx1] vector with either the horizontal (bu) or vertical (bv) flow.
  // x in the system are the parameters for the horizontal (pu) or vertical (pv) flow field.

  // local vars for iterating, random numbers:
  int sam, p, i_rand, si, add_si;

  // ensure that n_samples is high enough to ensure a result for a single fit:
  n_samples = (n_samples < MIN_SAMPLES_FIT) ? MIN_SAMPLES_FIT : n_samples;
  // n_samples should not be higher than count:
  n_samples = (n_samples < count) ? n_samples : count;

  // initialize matrices and vectors for the full point set problem:
  // this is used for determining inliers
  float _AA[count][3];
  MAKE_MATRIX_PTR(AA, _AA, count);
  float _bu_all[count][1];
  MAKE_MATRIX_PTR(bu_all, _bu_all, count);
  float _bv_all[count][1];
  MAKE_MATRIX_PTR(bv_all, _bv_all, count);
  for (sam = 0; sam < count; sam++) {
    AA[sam][0] = (float) vectors[sam].pos.x;
    AA[sam][1] = (float) vectors[sam].pos.y;
    AA[sam][2] = 1.0f;
    bu_all[sam][0] = (float) vectors[sam].flow_x;
    bv_all[sam][0] = (float) vectors[sam].flow_y;
  }

  // later used to determine the error of a set of parameters on the whole set:
  float _bb[count][1];
  MAKE_MATRIX_PTR(bb, _bb, count);
  float _C[count][1];
  MAKE_MATRIX_PTR(C, _C, count);

  // ***************
  // perform RANSAC:
  // ***************

  // set up variables for small linear system solved repeatedly inside RANSAC:
  float _A[n_samples][3];
  MAKE_MATRIX_PTR(A, _A, n_samples);
  float _bu[n_samples][1];
  MAKE_MATRIX_PTR(bu, _bu, n_samples);
  float _bv[n_samples][1];
  MAKE_MATRIX_PTR(bv, _bv, n_samples);
  float w[n_samples], _v[3][3];
  MAKE_MATRIX_PTR(v, _v, 3);
  float _pu[3][1];
  MAKE_MATRIX_PTR(pu, _pu, 3);
  float _pv[3][1];
  MAKE_MATRIX_PTR(pv, _pv, 3);

  // iterate and store parameters, errors, inliers per fit:
  float PU[n_iterations * 3];
  float PV[n_iterations * 3];
  float errors_pu[n_iterations];
  errors_pu[0] = 0.0;
  float errors_pv[n_iterations];
  errors_pv[0] = 0.0;
  int n_inliers_pu[n_iterations];
  int n_inliers_pv[n_iterations];
  int it, ii;
  for (it = 0; it < n_iterations; it++) {
    // select a random sample of n_sample points:
    int sample_indices[n_samples];
    i_rand = 0;

    // sampling without replacement:
    while (i_rand < n_samples) {
      si = rand() % count;
      add_si = 1;
      for (ii = 0; ii < i_rand; ii++) {
        if (sample_indices[ii] == si) { add_si = 0; }
      }
      if (add_si) {
        sample_indices[i_rand] = si;
        i_rand ++;
      }
    }

    // Setup the system:
    for (sam = 0; sam < n_samples; sam++) {
      A[sam][0] = (float) vectors[sample_indices[sam]].pos.x;
      A[sam][1] = (float) vectors[sample_indices[sam]].pos.y;
      A[sam][2] = 1.0f;
      bu[sam][0] = (float) vectors[sample_indices[sam]].flow_x;
      bv[sam][0] = (float) vectors[sample_indices[sam]].flow_y;
      //printf("%d,%d,%d,%d,%d\n",A[sam][0],A[sam][1],A[sam][2],bu[sam][0],bv[sam][0]);
    }

    // Solve the small system:

    // for horizontal flow:
    // decompose A in u, w, v with singular value decomposition A = u * w * vT.
    // u replaces A as output:
    pprz_svd_float(A, w, v, n_samples, 3);
    pprz_svd_solve_float(pu, A, w, v, bu, n_samples, 3, 1);
    PU[it * 3] = pu[0][0];
    PU[it * 3 + 1] = pu[1][0];
    PU[it * 3 + 2] = pu[2][0];

    // for vertical flow:
    pprz_svd_solve_float(pv, A, w, v, bv, n_samples, 3, 1);
    PV[it * 3] = pv[0][0];
    PV[it * 3 + 1] = pv[1][0];
    PV[it * 3 + 2] = pv[2][0];

    // count inliers and determine their error on all points:
    errors_pu[it] = 0;
    errors_pv[it] = 0;
    n_inliers_pu[it] = 0;
    n_inliers_pv[it] = 0;

    // for horizontal flow:
    // bb = AA * pu:
    MAT_MUL(count, 3, 1, bb, AA, pu);
    // subtract bu_all: C = 0 in case of perfect fit:
    MAT_SUB(count, 1, C, bb, bu_all);

    for (p = 0; p < count; p++) {
      C[p][0] = abs(C[p][0]);
      if (C[p][0] < error_threshold) {
        errors_pu[it] += C[p][0];
        n_inliers_pu[it]++;
      } else {
        errors_pu[it] += error_threshold;
      }
    }

    // for vertical flow:
    // bb = AA * pv:
    MAT_MUL(count, 3, 1, bb, AA, pv);
    // subtract bv_all: C = 0 in case of perfect fit:
    MAT_SUB(count, 1, C, bb, bv_all);

    for (p = 0; p < count; p++) {
      C[p][0] = abs(C[p][0]);
      if (C[p][0] < error_threshold) {
        errors_pv[it] += C[p][0];
        n_inliers_pv[it]++;
      } else {
        errors_pv[it] += error_threshold;
      }
    }
  }

  // After all iterations:
  // select the parameters with lowest error:
  // for horizontal flow:
  int param;
  int min_ind = 0;
  *min_error_u = (float)errors_pu[0];
  for (it = 1; it < n_iterations; it++) {
    if (errors_pu[it] < *min_error_u) {
      *min_error_u = (float)errors_pu[it];
      min_ind = it;
    }
  }
  for (param = 0; param < 3; param++) {
    parameters_u[param] = PU[min_ind * 3 + param];
  }
  *n_inliers_u = n_inliers_pu[min_ind];

  // for vertical flow:
  min_ind = 0;
  *min_error_v = (float)errors_pv[0];
  for (it = 0; it < n_iterations; it++) {
    if (errors_pv[it] < *min_error_v) {
      *min_error_v = (float)errors_pv[it];
      min_ind = it;
    }
  }
  for (param = 0; param < 3; param++) {
    parameters_v[param] = PV[min_ind * 3 + param];
  }
  *n_inliers_v = n_inliers_pv[min_ind];

  // error has to be determined on the entire set without threshold:
  // bb = AA * pu:
  MAT_MUL(count, 3, 1, bb, AA, pu);
  // subtract bu_all: C = 0 in case of perfect fit:
  MAT_SUB(count, 1, C, bb, bu_all);
  *min_error_u = 0;
  for (p = 0; p < count; p++) {
    *min_error_u += abs(C[p][0]);
  }
  // bb = AA * pv:
  MAT_MUL(count, 3, 1, bb, AA, pv);
  // subtract bv_all: C = 0 in case of perfect fit:
  MAT_SUB(count, 1, C, bb, bv_all);
  *min_error_v = 0;
  for (p = 0; p < count; p++) {
    *min_error_v += abs(C[p][0]);
  }
  *fit_error = (*min_error_u + *min_error_v) / (2 * count);

}
/**
 * Extract information from the parameters that were fit to the optical flow field.
 * @param[in] parameters_u* Parameters of the horizontal flow field
 * @param[in] parameters_v* Parameters of the vertical flow field
 * @param[in] im_width Width of image in pixels
 * @param[in] im_height Height of image in pixels
 * @param[out] info Contains all info extracted from the linear flow fit
 */

void extract_information_from_parameters(float *parameters_u, float *parameters_v, int im_width, int im_height, struct linear_flow_fit_info *info)
{
  // This method assumes a linear flow field in x- and y- direction according to the formulas:
  // u = parameters_u[0] * x + parameters_u[1] * y + parameters_u[2]
  // v = parameters_v[0] * x + parameters_v[1] * y + parameters_v[2]
  // where u is the horizontal flow at image coordinate (x,y)
  // and v is the vertical flow at image coordinate (x,y)

  // relative velocities:
  info->relative_velocity_z = (parameters_u[0] + parameters_v[1]) / 2.0f; // divergence / 2

  // translation orthogonal to the camera axis:
  // flow in the center of the image:
  info->relative_velocity_x = -(parameters_u[2] + (im_width / 2.0f) * parameters_u[0] + (im_height / 2.0f) * parameters_u[1]);
  info->relative_velocity_y = -(parameters_v[2] + (im_width / 2.0f) * parameters_v[0] + (im_height / 2.0f) * parameters_v[1]);

  float arv_x = abs(info->relative_velocity_x);
  float arv_y = abs(info->relative_velocity_y);

  // extract inclination from flow field:
  float threshold_slope = 1.0;
  float eta = 0.002;

  if (abs(parameters_v[1]) < eta && arv_y < threshold_slope && arv_x >= 2 * threshold_slope) {
    // there is no forward motion and not enough vertical motion, but enough horizontal motion:
    info->slope_x = parameters_u[0] / info->relative_velocity_x;
  } else if (arv_y >= 2 * threshold_slope) {
    // there is sufficient vertical motion:
    info->slope_x = parameters_v[0] / info->relative_velocity_y;
  } else {
    // there may be forward motion, then we can do a quadratic fit:
    // a linear fit provides no information though
    info->slope_x = 0.0f;
  }

  if (abs(parameters_u[0]) < eta && arv_x < threshold_slope && arv_y >= 2 * threshold_slope) {
    // there is no forward motion, little horizontal movement, but sufficient vertical motion:
    info->slope_y = parameters_v[1] / info->relative_velocity_y;
  } else if (arv_x >= 2 * threshold_slope) {
    // there is sufficient horizontal motion:
    info->slope_y = parameters_u[1] / info->relative_velocity_x;
  } else {
    // there could be forward motion, then we can do a quadratic fit:
    // a linear fit provides no information though
    info->slope_y = 0.0f;
  }

  // Focus of Expansion:
  // the flow planes intersect the flow=0 plane in a line
  // the FoE is the point where these 2 lines intersect (flow = (0,0))
  // x:
  float denominator = parameters_v[0] * parameters_u[1] - parameters_u[0] * parameters_v[1];
  if (abs(denominator) > 1E-5) {
    info->focus_of_expansion_x = ((parameters_u[2] * parameters_v[1] - parameters_v[2] * parameters_u[1]) / denominator);
  } else { info->focus_of_expansion_x = 0.0f; }
  // y:
  denominator = parameters_u[1];
  if (abs(denominator) > 1E-5) {
    info->focus_of_expansion_y = (-(parameters_u[0] * (info->focus_of_expansion_x) + parameters_u[2]) / denominator);
  } else { info->focus_of_expansion_y = 0.0f; }
}

