#ifndef HUMANLIKE_H
#define HUMANLIKE_H

#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "coordinateconversions.h"
#include "arrayfunctions.h"

void hl_getnextpos( float *px, float *py, float vx, float vy, float dt);

float hl_distbetweenpoints(float p1, float p2, float r1, float r2);

int hl_collisiontest( float px, float py, 
	float vx_own, float vy_own,
	float vx_obst, float vy_obst,
	float dt, float radius );

int hl_prospective( float *vec, float px, float py, 
	float vx_own, float vy_own,
	float vx_obst, float vy_obst,
	float dt , float max, int length, float radius );

float hl_selectangle(int length, float *psi_des_vec);

#endif
