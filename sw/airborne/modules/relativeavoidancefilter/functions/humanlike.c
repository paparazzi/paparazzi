#include "humanlike.h"

void hl_getnextpos( float *px, float *py, 
	float vx, float vy, float dt )
{
	*px = *px + vx*dt;
	*py = *py + vy*dt;
}


float hl_distbetweenpoints(float p1, float p2, float r1, float r2)
{
	return pow((p1 - r1),2) + pow((p2 - r2),2);
}


int hl_collisiontest( float px, float py, 
	float vx_own, float vy_own,
	float vx_obst, float vy_obst,
	float dt, float radius )
{	
	float pox = 0.0;
	float poy = 0.0;
	float pobx = px;
	float poby = py;

	hl_getnextpos(&pox, &poy, vx_own, vy_own, dt);
	hl_getnextpos(&pobx, &poby, vx_obst, vy_obst, dt);

	/* Check if over the horizon interval dt
	there will be any sign of a collision */
	if (hl_distbetweenpoints(pox, poy, pobx, poby) > radius)
		return 0; // no collision
	else 
		return 1; // collision
}


int hl_prospective( float *vec, float px, float py, 
	float vx_own, float vy_own,
	float vx_obst, float vy_obst,
	float dt, float max, int length, float radius)
{
	int i, j;
	float v, ang, vox, voy, temp;
	cart2polar(vx_own, vy_own, &v, &temp);
	int flag = 0;

	for (i = 0; i < length; i++)
	{

		j = 1;
		ang = -M_PI + (2*M_PI/length)*i;
		wrapToPi(&ang);
		polar2cart(v, ang, &vox, &voy);

		while(j*dt < max)
		{

			vec[i] = j*dt;
			flag = hl_collisiontest( px, py, vox, voy, vx_obst, vy_obst, j*dt, radius);

			if (flag)
				break;
			else
				j++;
			
		}

	}

	return flag;

}

float hl_selectangle(int length, float *psi_des_vec)
{	
	int idx;
	idx = array_getmaxidx(length, psi_des_vec);
	//printf("getmaxidx: %d \n", idx);
	return psi_des_vec[idx];
}