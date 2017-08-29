#ifndef RANDOMGENERATOR_H
#define RANDOMGENERATOR_H

#include "stdlib.h"
#include "math.h"

/* Initilizer function.

Run this at the beginning of the code and before running any of the functions  below.
It will create an initial random seed that is used later on, tuned to the current time AND to the process ID
(this is because it was found that only using the time is not sufficient, and two nodes spawned at the same time will actually end with the same seed unless the process ID is also included).
*/
extern void randomgen_init(void);

/* Get a random value of type float between a min and max */
extern float getrand_float(float min, float max);

/* Get a random value of type int between a min and a max */
extern int getrand_int(int min, int max);

/* Get a random value of type int between a min and a max */
extern float rand_normal(float mean, float stddev);

#endif