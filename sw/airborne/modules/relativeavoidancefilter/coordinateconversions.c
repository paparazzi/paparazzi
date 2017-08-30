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
 * @file "modules/relativeavoidancefilter/coordinateconversions.c"
 * @author Mario Coppola
 * Functions to convert between coordinate frames
 */

#include "coordinateconversions.h"

/* Function to convert polar coordinates to cartesian*/
void polar2cart(float radius, float radians, float *x, float *y)
{
	wrapToPi(&radians);
	*x = radius * cos(radians); 
	*y = radius * sin(radians);
};

/* Function to convert cartesian coordinates to polar */
void cart2polar(float x, float y, float *radius, float *radians)
{
	*radius  = sqrt(pow(x,2) + pow(y,2));
	*radians = atan2(y,x);
};

/* Function to convert radians to degrees */
void rad2deg(float rad, float *deg)
{

	*deg = 180.0 / M_PI * rad;
}

/* Function to convert degrees to radians */
void deg2rad(float deg, float *rad)
{
	*rad = M_PI / 180.0 * deg;
}

/* Wraps an angle in radians between -PI and +PI */
void wrapToPi(float *ang)
{
	if (*ang > M_PI) {
		while (*ang > M_PI) {		
			*ang = *ang - 2*M_PI;
		}
	}	
	else if (*ang < -M_PI) {
		while (*ang < -M_PI) {
			*ang = *ang + 2*M_PI;
		}
	}
}

/* Wraps an angle in radians between 0 and 2PI */
void wrapTo2Pi(float *ang)
{
	if (*ang > 2*M_PI) {
		while (*ang > 2*M_PI) {		
			*ang = *ang - 2*M_PI;
		}
	}

	else if (*ang < 0.0) {
		while (*ang < 0.0) {
			*ang = *ang + 2*M_PI;
		}
	}
}

/* Keeps a value between two bounds */
void keepbounded(float *value, float min, float max)
{
	if (*value < min) {		
		*value = min;
	}
	else if (*value > max) {
		*value = max;
	}
}


void GazeboToBody(float xe, float ye, float psi, float *xb, float *yb)
{
	*xb = xe*cos(psi) - ye*sin(psi);
	*yb = - xe*sin(psi) - ye*cos(psi);
}

void BodyToGazebo(float xb, float yb, float psi, float *xe, float *ye)
{
	// *xe = xb*cos(-psi) - yb*sin(-psi);
	// *ye = - xb*sin(-psi) - yb*cos(-psi);
	*xe = xb*cos(psi) + yb*sin(psi);
	*ye = -xb*sin(psi) + yb*cos(psi);
}		

void BodyToNED(float xb, float yb, float psi, float *xe, float *ye)
{
	*xe = xb*cos(psi) + yb*sin(psi);
	*ye = -xb*sin(psi) + yb*cos(psi);
}		