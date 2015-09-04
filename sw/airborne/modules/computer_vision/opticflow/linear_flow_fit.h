/*
 * Copyright (C) 2015 de Croon, Hann Woei Ho
 *	
 * Algorithm introduced and explained in: 
 * (2013), "Optic-flow based slope estimation for autonomous landing", de Croon, G.C.H.E., and Ho, H.W., and De Wagter, C., and van Kampen,  
 * E., and Remes B., and Chu, Q.P., in the International Journal of Micro Air Vehicles, Volume 5, Number 4, pages 287 – 297.	 
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
 * @file paparazzi/sw/airborne/modules/computer_vision/opticflow/linear_flow_fit.h
 * @brief Takes a set of optical flow vectors and extracts information such as relative velocities and surface roughness.
 *
 * A horizontal and vertical linear fit is made with the optical flow vectors, and from the fit parameters information is extracted such as relative velocities (useful for time-to-contact determination), slope angle, and surface roughness.
 */

#include "lib/vision/image.h"

#ifndef LINEAR_FLOW_FIT
#define LINEAR_FLOW_FIT

// This is the function called externally, passing the vector of optical flow vectors and information on the number of vectors and image size: 
int analyze_linear_flow_field(struct flow_t* vectors, int count, float error_threshold, int n_iterations, int n_samples, int im_width, int im_height, float *slope_x, float *slope_y, float *surface_roughness, float *focus_of_expansion_x, float *focus_of_expansion_y, float *relative_velocity_x, float *relative_velocity_y, float *relative_velocity_z, float *time_to_contact, float* divergence, float *fit_error, int *n_inliers_u, int *n_inliers_v);

// Fits the linear flow field with RANSAC:
void fit_linear_flow_field(struct flow_t* vectors, int count, float error_threshold, int n_iterations, int n_samples, float* parameters_u, float* parameters_v, float* fit_error, float* min_error_u, float* min_error_v, int *n_inliers_u, int *n_inliers_v);

// Extracts relevant information from the fit parameters:
void extract_information_from_parameters(float* parameters_u, float* parameters_v, int im_width, int im_height, float *relative_velocity_x, float *relative_velocity_y, float *relative_velocity_z, float *slope_x, float *slope_y, float *focus_of_expansion_x, float *focus_of_expansion_y);


#endif
