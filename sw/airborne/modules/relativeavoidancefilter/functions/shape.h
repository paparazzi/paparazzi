#ifndef SHAPE_H
#define SHAPE_H

#include "string.h"
#include "math.h"
#include "arrayfunctions.h"

extern float shape_getarea(float *tr, int size);

extern void shape_rotateatorigin(float *tr, int size, float ang);

extern void shape_rotateatpoint(float *tr, int size, float ang, float x0, float y0);
	
extern void shape_shift(float *tr, int size, float x_shift, float y_shift);

extern bool shape_checkifpointinarea(float* tr, int size, float *px);

#endif