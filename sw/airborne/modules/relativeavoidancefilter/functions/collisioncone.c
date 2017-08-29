#include "collisioncone.h"
#define PSISEARCH 10

// STANDARD IMPLEMENTATION
/*
	Updated the coordinates of the triangle for the collision cone of an obstacle.
*/
void collisioncone_update( float *cc,
	float relx, float rely,
	float relvx, float relvy,
	float radius)
{
	float range, bearing;
	cart2polar(relx, rely, &range, &bearing);

	// The other two edges are symmetrical about a diagonal extending along the localization bearing.
	cc[0] = 0.0;
	cc[1] = 0.0;

	// This is the edge extending upwards to the right (+y)
	cc[2] = 5.0; 							   // x_body
	cc[3] = 5.0*tan(atan(radius/(1.0*range))); // y_body

	// This is the edge exteding upwards to the left (-y)
	cc[4] = cc[2]; 	 // x_body
	cc[5] = -cc[3];  // y_body

	shape_rotateatorigin(cc, 6, bearing);
	shape_shift(cc, 6, relvx, relvy);

}

bool collisioncone_checkdanger( float *cc, float ownvx, float ownvy)
{
	float vv[2];
	vv[0] = ownvx;
	vv[1] = ownvy;

	// Check if the current velocity is ok.
	// If point is in area then we get a flag
	return shape_checkifpointinarea(cc, 6, vv);
};


void collisioncone_findnewcmd( float cc[2][6], 
	float *v_des, float *psi_des, 
	float psisearch, int nfilters )
{
	int i;
	int count = 1;
	// int ng = 1;
	bool flag = true;
	float psi_add;
	deg2rad(psisearch, &psi_add);

	float psi0 = *psi_des;
	float vx, vy;

	while (*v_des <= 1.0) {
		polar2cart(*v_des, *psi_des, &vx, &vy);

		for (i = 0; i < nfilters; i++) { // Check if we succeed
			flag = collisioncone_checkdanger(cc[i], vx, vy);
			if (flag)
				break;
		}

		if(!flag) // No issues found
			return;

		*psi_des = psi0 + (count * psi_add);
		wrapTo2Pi(psi_des);

		// ng = ng * -1;
		count++;
			
		if (count >= (2*M_PI)/psi_add) {
			*v_des = *v_des + *v_des;
			count = 1;
		}
	}
	
	// Failed to find a solution, so just stop this time
	*v_des = 0.0;
};

float movingaveragefilter(float *vec, int size, float newelement)
{
	array_shiftleft(vec, size, 1);
	vec[size-1] = newelement;
	float out = array_sum(size, vec)/(float)size;
	vec[size-1] = out;
	return out;
}


///// BOOLEAN IMPLEMENTATION FUNCTIONS

void setboolinrange( bool *cc, float lb, float up, int d){
	int i;
		if (lb>up){ // crosses over 360 line
			for (i = 0.0; i < d; i++) {
				if (((PSISEARCH*i) > lb) || ((PSISEARCH*i) < up))
					cc[i] = true;
			}
		}

		else{
			for (i = 0; i < d; i++) {
				if (((PSISEARCH*i) > lb) && ((PSISEARCH*i) < up))
					cc[i] = true;
			}
		}
}

/*
	Updated the coordinates of the triangle for the collision cone of an obstacle.
*/
void collisioncone_update_bool( bool *cc,
	float relx, float rely,
	float radius)
{
	float range, bearing;
	cart2polar(relx, rely, &range, &bearing);

	float lb = bearing - atan(radius/(1.0*range)); // y_body
	float up = bearing + atan(radius/(1.0*range));
	wrapTo2Pi(&lb);
	wrapTo2Pi(&up);

	lb = 180.0 / M_PI * lb;
	up = 180.0 / M_PI * up;

	setboolinrange(cc, lb, up, 36);
}

bool collisioncone_findnewdir_bool( bool *cc, float *psi_des)
{
	int i;
	float temp = *psi_des;
	*psi_des = 180.0 / M_PI * *psi_des; // convert to degrees
	float p = *psi_des/PSISEARCH; // round it here
	array_shiftleft_bool(cc, 36, (int)p);
	
	for (i = 0; i < 36; i++)
	{
		if (!cc[i]){
			*psi_des = (i+(int)p)*PSISEARCH;
			*psi_des = M_PI / 180.0 * *psi_des; // convert to radians
			wrapToPi(psi_des);
			break;
		}
	}

	if (abs(temp - *psi_des) < 0.01) // check equal but with rounding
		return false;
	else
		return true;

};
