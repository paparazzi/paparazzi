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

#ifndef RANSAC_H
#define RANSAC_H

#ifdef __cplusplus
extern "C" {
#endif

#include "std.h"

/** Perform RANSAC to fit a linear model.
 *
 * @param[in] n_samples The number of samples to use for a single fit
 * @param[in] n_iterations The number of times a linear fit is performed
 * @param[in] error_threshold The threshold used to cap errors in the RANSAC process
 * @param[in] targets The target values
 * @param[in] samples The samples / feature vectors
 * @param[in] D The dimensionality of the samples
 * @param[in] count The number of samples
 * @param[in] use_bias Whether the RANSAC procedure should add a bias. If 0 it does not.
 * @param[out] parameters* Parameters of the linear fit
 * @param[out] fit_error* Total error of the fit
 */
void RANSAC_linear_model(int n_samples, int n_iterations, float error_threshold, float *targets, int D,
                         float (*samples)[D], uint16_t count, bool use_bias, float *params, float *fit_error);

/** Get indices without replacement.
 *
 * @param[out] indices_subset This will be filled with the sampled indices
 * @param[in] n_samples The number of samples / indices.
 * @param[in] count The function will sample n_sample numbers from the range 1, 2, 3,..., count
 */
void get_indices_without_replacement(int *indices_subset, int n_samples, int count);

/** Predict the value of a sample with linear weights.
 *
 * @param[in] sample The sample vector of size D
 * @param[in] weights The weight vector of size D+1
 * @param[in] D The dimension of the sample.
 * @return The predicted value
 */
float predict_value(float *sample, float *weights, int D, bool use_bias);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* RANSAC_H */
