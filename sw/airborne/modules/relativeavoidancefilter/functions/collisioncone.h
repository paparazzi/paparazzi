#ifndef COLLISIONCONE_H
#define COLLISIONCONE_H

#include "math.h"
#include "shape.h"
#include "coordinateconversions.h"

// STANDARD
void collisioncone_update( float *cc,
	float relx, float rely,
	float relvx, float relvy,
	float radius);

bool collisioncone_checkdanger( float *cc, 
	float ownvx, float ownvy);

void collisioncone_findnewcmd(	float cc[][6], 
	float *v_des, float *psi_des,
	float psisearch, int nfilters );
	
float movingaveragefilter(float *array, int size, float newelement);

// BOOLEAN
void collisioncone_update_bool( bool *cc,
	float relx, float rely,
	float radius);
	
bool collisioncone_findnewdir_bool(	bool *cc, float *psi_des);

#endif