#include "randomgenerator.h"
#include "stdint.h"
/* Initilizer function.

Run this at the beginning of the code and before running any of the functions  below.
It will create an initial random seed that is used later on, tuned to the current time AND to the process ID
(this is because it was found that only using the time is not sufficient, and two nodes spawned at the same time will actually end with the same seed unless the process ID is also included).
*/
void randomgen_init()
{
	int temp;
	uintptr_t t = (uintptr_t)&temp;
	
	srand( t );
}

/* Get a random value of type float between a min and max */
float getrand_float(float min, float max)
{
	return min + ((float)rand() / ( RAND_MAX / (max - min) ) ) ;
};

/* Get a random value of type int between a min and a max */
int getrand_int(int min, int max)
{
	return min + (rand() / ( RAND_MAX / (max - min) ) ) ;
};

// <<complete Box-Muller function>>=
// based on:
//http://en.literateprograms.org/Box-Muller_transform_%28C%29#chunk%20def:scale%20and%20translate%20to%20get%20desired%20mean%20and%20standard%20deviation
//
// Polar form of Box-Muller transform:
// http://www.design.caltech.edu/erik/Misc/Gaussian.html
float rand_normal(float mean, float stddev) {
	static float n2 = 0.0;
	static int n2_cached = 0;
	if (!n2_cached) {
		float x, y, r;
		do {
			x = 2.0*rand()/RAND_MAX - 1;
			y = 2.0*rand()/RAND_MAX - 1;
			r = x*x + y*y;
		}  //
		while (r == 0.0 || r > 1.0); {
			float d = sqrt(-2.0*log(r)/r);
			float n1 = x*d;
			n2 = y*d;
			float result = n1*stddev + mean;
			n2_cached = 1;
			return result;
		}
	} 
	else {
		n2_cached = 0;
		return n2*stddev + mean;
	}

}