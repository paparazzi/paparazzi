/*
 * Copyright (C) 2018 Guido de Croon
 *
 * This file is part of paparazzi.
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */

/**
 * @file RANSAC.h
 * @brief Perform Random Sample Consensus (RANSAC), a robust fitting method.
 *
 * The concept is to select (minimal) subsets of the samples for learning
 * a model, then determining the error on the entire data set, and selecting the best fit. In determining the error, the individual sample errors
 * are capped, so that outliers can be identified and have only a modest influence on the results.
 *
 * Read: Fischler, M. A., & Bolles, R. C. (1981). Random sample consensus: a paradigm for model fitting with applications to image analysis and automated cartography.
 * Communications of the ACM, 24(6), 381-395.
 *
 * This file depends on the function fit_linear_model in math/pprz_matrix_decomp_float.h/c
 */


#include "RANSAC.h"
#include "math/pprz_matrix_decomp_float.h"
#include "math/pprz_algebra_float.h"
#include <math.h>
#include <string.h>
#include <stdlib.h>
#include "stdio.h"

/** Perform RANSAC to fit a linear model.
 *
 * @param[in] n_samples The number of samples to use for a single fit
 * @param[in] n_iterations The number of times a linear fit is performed
 * @param[in] error_threshold The threshold used to cap errors in the RANSAC process
 * @param[in] targets The target values
 * @param[in] samples The samples / feature vectors
 * @param[in] D The dimensionality of the samples
 * @param[in] count The number of samples
 * @param[out] parameters* Parameters of the linear fit, of size D + 1 (accounting for a constant 1 being added to the samples to represent a potential bias)
 * @param[out] fit_error* Total error of the fit
 *
 */
void RANSAC_linear_model(int n_samples, int n_iterations, float error_threshold, float *targets, int D,
                         float (*samples)[D], uint16_t count, float *params, float *fit_error __attribute__((unused)))
{

  uint8_t D_1 = D + 1;
  float err;
  float errors[n_iterations];
  int indices_subset[n_samples];
  float subset_targets[n_samples];
  float subset_samples[n_samples][D];
  float subset_params[n_iterations][D_1];
  bool use_bias = true;

  // ensure that n_samples is high enough to ensure a result for a single fit:
  n_samples = (n_samples < D_1) ? D_1 : n_samples;
  // n_samples should not be higher than count:
  n_samples = (n_samples < count) ? n_samples : count;

  // do the RANSAC iterations:
  for (int i = 0; i < n_iterations; i++) {

    // get a subset of indices
    get_indices_without_replacement(indices_subset, n_samples, count);

    // get the corresponding samples and targets:
    for (int j = 0; j < n_samples; j++) {
      subset_targets[j] = targets[indices_subset[j]];
      for (int k = 0; k < D; k++) {
        subset_samples[j][k] = samples[indices_subset[j]][k];
      }
    }

    // fit a linear model on the small system:
    fit_linear_model(subset_targets, D, subset_samples, n_samples, use_bias, subset_params[i], &err);
    printf("params normal: %f, %f\n", subset_params[i][0], subset_params[i][1]);
    float priors[2];
    priors[0] = 1.0f;
    priors[1] = 10.0f;
    fit_linear_model_prior(subset_targets, D, subset_samples, n_samples, use_bias, priors, subset_params[i], &err);
    printf("params prior: %f, %f\n", subset_params[i][0], subset_params[i][1]);

    // determine the error on the whole set:
    float err_sum = 0.0f;
    float prediction;
    for (int j = 0; j < count; j++) {
      // predict the sample's value and determine the absolute error:
      prediction = predict_value(samples[j], subset_params[i], D, use_bias);
      err = fabsf(prediction - targets[j]);
      // cap the error with the threshold:
      err = (err > error_threshold) ? error_threshold : err;
      err_sum += err;
    }
    errors[i] = err_sum;
  }

  // determine the minimal error:
  float min_err = errors[0];
  int min_ind = 0;
  for (int i = 1; i < n_iterations; i++) {
    if (errors[i] < min_err) {
      min_err = errors[i];
      min_ind = i;
    }
  }

  // copy the parameters:
  for (int d = 0; d < D_1; d++) {
    params[d] = subset_params[min_ind][d];
  }

}

/** Predict the value of a sample with linear weights.
 *
 * @param[in] sample The sample vector of size D
 * @param[in] weights The weight vector of size D+1
 * @param[in] D The dimension of the sample.
 * @return The predicted value
 */
float predict_value(float *sample, float *weights, int D, bool use_bias)
{

  float sum = 0.0f;

  for (int w = 0; w < D; w++) {
    sum += weights[w] * sample[w];
  }
  if (use_bias) {
    sum += weights[D];
  }

  // printf("Prediction = %f\n", sum);

  return sum;
}

/** Get indices without replacement.
 *
 * @param[out] indices_subset This will be filled with the sampled indices
 * @param[in] n_samples The number of samples / indices.
 * @param[in] count The function will sample n_sample numbers from the range 1, 2, 3,..., count
 */

void get_indices_without_replacement(int *indices_subset, int n_samples, int count)
{

  int index;

  for (int j = 0; j < n_samples; j++) {
    bool picked_number = false;
    while (!picked_number) {
      index = rand() % count;
      bool new_val = true;
      for (int k = 0; k < j; k++) {
        if (indices_subset[k] == index) {
          new_val = false;
          break;
        }
      }
      if (new_val) {
        indices_subset[j] = index;
        picked_number = true;
      }
    }
  }
}
