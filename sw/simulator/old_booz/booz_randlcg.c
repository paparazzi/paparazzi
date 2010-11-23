/* rndlcg            Linear Congruential Method, the "minimal standard generator"
                     Park & Miller, 1988, Comm of the ACM, 31(10), pp. 1192-1201

*/

#include <math.h>
#include <limits.h>

#define ALL_BITS     0xffffffff

static long int quotient  = LONG_MAX / 16807L;
static long int my_remainder = LONG_MAX % 16807L;

static long int seed_val = 1L;

long set_seed(long int sd)
{
  return seed_val = sd;
}

long get_seed()
{
  return seed_val;
}


unsigned long int randlcg()       /* returns a random unsigned integer */
{
  if ( seed_val <= quotient )
    seed_val = (seed_val * 16807L) % LONG_MAX;
  else
    {
      long int high_part = seed_val / quotient;
      long int low_part  = seed_val % quotient;

      long int test = 16807L * low_part - my_remainder * high_part;

      if ( test > 0 )
	seed_val = test;
      else
	seed_val = test + LONG_MAX;

    }

  return seed_val;
}





