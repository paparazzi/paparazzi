/*
 * Copyright (C) Mario Coppola
 *
 * This file is part of paparazzi
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
 * @file "modules/relativelocalizationfilter/coordinateconversions.h"
 * @author Mario Coppola
 * Functions to convert between coordinate frames
 */

#ifndef COORDINATECONVERSIONS_H
#define COORDINATECONVERSIONS_H

// Standard C includes
#include "math.h"

/* Function to convert polar coordinates to cartesian*/
void polar2cart(float radius, float radians, float *x, float *y);

/* Function to convert cartesian coordinates to polar */
void cart2polar(float x, float y, float *radius, float *radians);

/* Function to convert radians to deg rees */
void rad2deg(float rad, float *deg);

/* Function to convert degrees to radians */
void deg2rad(float deg, float *rad);

/* Wraps an angle in radians between -PI and +PI */
void wrapToPi(float *ang);

/* Wraps an angle in radians between 0 and 2PI */
void wrapTo2Pi(float *ang);

/* Keeps a value between two bounds */
void keepbounded(float *value, float min, float max);

/* Planar conversions between Body frame and Gazebo world frame */
void GazeboToBody(float xe, float ye, float psi, float *xb, float *yb);
void BodyToGazebo(float xb, float yb, float psi, float *xe, float *ye);
void BodyToNED(float xb, float yb, float psi, float *xe, float *ye);

#endif