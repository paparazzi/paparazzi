#ifndef COORDINATECONVERSIONS_H
#define COORDINATECONVERSIONS_H

// Standard C includes
#include "math.h"

/* Function to convert polar coordinates to cartesian

Compliments of 
http://stackoverflow.com/questions/29089710/pointers-in-c-programming-coordinate-conversion */
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