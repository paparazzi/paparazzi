/*
 * kalman_filter_vision.h
 *
 *  Created on: Sep 12, 2016
 *      Author: knmcguire
 */

#ifndef KALMAN_FILTER_VISION_H_
#define KALMAN_FILTER_VISION_H_

void kalman_filter_linear_2D_float(float *model, float *measurements, float *covariance, float *state,
		float *process_noise, float *measurement_noise);


#endif /* KALMAN_FILTER_VISION_H_ */
