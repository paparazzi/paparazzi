#include "nps_random.h"


#include <math.h>
#include <limits.h>


#if 0
/*
 * R250
 * Kirkpatrick, S., and E. Stoll, 1981; "A Very Fast
 * Shift-Register Sequence Random Number Generator",
 * Journal of Computational Physics, V.40
 *
 */

static void         r250_init(int seed);
//static unsigned int r250( void );
static double       dr250(void);

/*
 * randclg
 * Linear Congruential Method, the "minimal standard generator"
 * Park & Miller, 1988, Comm of the ACM, 31(10), pp. 1192-1201
 *
 */

static long              set_seed(long);
//static long              get_seed(void);
static unsigned long int randlcg(void);
#endif


void double_vect3_add_gaussian_noise(struct DoubleVect3 *vect, struct DoubleVect3 *std_dev)
{
  vect->x += get_gaussian_noise() * std_dev->x;
  vect->y += get_gaussian_noise() * std_dev->y;
  vect->z += get_gaussian_noise() * std_dev->z;
}

void float_vect3_add_gaussian_noise(struct FloatVect3 *vect, struct FloatVect3 *std_dev)
{
  vect->x += get_gaussian_noise() * std_dev->x;
  vect->y += get_gaussian_noise() * std_dev->y;
  vect->z += get_gaussian_noise() * std_dev->z;
}

void float_rates_add_gaussian_noise(struct FloatRates *vect, struct FloatRates *std_dev)
{
  vect->p += get_gaussian_noise() * std_dev->p;
  vect->q += get_gaussian_noise() * std_dev->q;
  vect->r += get_gaussian_noise() * std_dev->r;
}



void double_vect3_get_gaussian_noise(struct DoubleVect3 *vect, struct DoubleVect3 *std_dev)
{
  vect->x = get_gaussian_noise() * std_dev->x;
  vect->y = get_gaussian_noise() * std_dev->y;
  vect->z = get_gaussian_noise() * std_dev->z;
}


void double_vect3_update_random_walk(struct DoubleVect3 *rw, struct DoubleVect3 *std_dev, double dt, double thau)
{
  struct DoubleVect3 drw;
  double_vect3_get_gaussian_noise(&drw, std_dev);
  struct DoubleVect3 tmp;
  VECT3_SMUL(tmp, *rw, (-1. / thau));
  VECT3_ADD(drw, tmp);
  VECT3_SMUL(drw, drw, dt);
  VECT3_ADD(*rw, drw);
}





#if 0
/*
   http://www.taygeta.com/random/gaussian.html
*/
double get_gaussian_noise(void)
{

  double x1;
  static int nb_call = 0;
  static double x2, w;
  if (nb_call == 0) { r250_init(0); }
  nb_call++;
  if (nb_call % 2) {
    do {
      x1 = 2.0 * dr250() - 1.0;
      x2 = 2.0 * dr250() - 1.0;
      w = x1 * x1 + x2 * x2;
    } while (w >= 1.0);

    w = sqrt((-2.0 * log(w)) / w);
    return x1 * w;
  } else {
    return x2 * w;
  }
}
#else
#include <gsl/gsl_rng.h>
#include <gsl/gsl_randist.h>
#include <stdlib.h>
double get_gaussian_noise(void)
{
  static gsl_rng *r = NULL;
  // select random number generator
  if (!r) { r = gsl_rng_alloc(gsl_rng_mt19937); }
  return gsl_ran_gaussian(r, 1.);
}
#endif


#if 0
/*
 * R250
 * Kirkpatrick, S., and E. Stoll, 1981; "A Very Fast
 * Shift-Register Sequence Random Number Generator",
 * Journal of Computational Physics, V.40
 *
 */

/* defines to allow for 16 or 32 bit integers */
#define BITS 31


#if WORD_BIT == 32
#ifndef BITS
#define BITS  32
#endif
#else
#ifndef BITS
#define BITS    16
#endif
#endif

#if BITS == 31
#define MSB          0x40000000L
#define ALL_BITS     0x7fffffffL
#define HALF_RANGE   0x20000000L
#define STEP         7
#endif

#if BITS == 32
#define MSB          0x80000000L
#define ALL_BITS     0xffffffffL
#define HALF_RANGE   0x40000000L
#define STEP         7
#endif

#if BITS == 16
#define MSB         0x8000
#define ALL_BITS    0xffff
#define HALF_RANGE  0x4000
#define STEP        11
#endif

static unsigned int r250_buffer[ 250 ];
static int r250_index;

static void r250_init(int sd)
{
  int j, k;
  unsigned int mask, msb;
  set_seed(sd);

  r250_index = 0;
  for (j = 0; j < 250; j++) {    /* fill r250 buffer with BITS-1 bit values */
    r250_buffer[j] = randlcg();
  }

  for (j = 0; j < 250; j++) /* set some MSBs to 1 */
    if (randlcg() > HALF_RANGE) {
      r250_buffer[j] |= MSB;
    }

  msb = MSB;          /* turn on diagonal bit */
  mask = ALL_BITS;  /* turn off the leftmost bits */

  for (j = 0; j < BITS; j++)  {
    k = STEP * j + 3; /* select a word to operate on */
    r250_buffer[k] &= mask; /* turn off bits left of the diagonal */
    r250_buffer[k] |= msb;  /* turn on the diagonal bit */
    mask >>= 1;
    msb  >>= 1;
  }

}

#if 0
/* returns a random unsigned integer */
static unsigned int r250(void)
{
  register int  j;
  register unsigned int new_rand;

  if (r250_index >= 147) {
    j = r250_index - 147;  /* wrap pointer around */
  } else {
    j = r250_index + 103;
  }

  new_rand = r250_buffer[ r250_index ] ^ r250_buffer[ j ];
  r250_buffer[ r250_index ] = new_rand;

  if (r250_index >= 249) { /* increment pointer for next time */
    r250_index = 0;
  } else {
    r250_index++;
  }

  return new_rand;

}
#endif

/* returns a random double in range 0..1 */
static double dr250(void)
{
  register int  j;
  register unsigned int new_rand;

  if (r250_index >= 147) {
    j = r250_index - 147;  /* wrap pointer around */
  } else {
    j = r250_index + 103;
  }

  new_rand = r250_buffer[ r250_index ] ^ r250_buffer[ j ];
  r250_buffer[ r250_index ] = new_rand;

  if (r250_index >= 249) { /* increment pointer for next time */
    r250_index = 0;
  } else {
    r250_index++;
  }

  return (double)new_rand / ALL_BITS;

}

/*
 * randclg
 * Linear Congruential Method, the "minimal standard generator"
 * Park & Miller, 1988, Comm of the ACM, 31(10), pp. 1192-1201
 *
 */

static long int quotient  = LONG_MAX / 16807L;
static long int my_remainder = LONG_MAX % 16807L;

static long int seed_val = 1L;

static long set_seed(long int sd)
{
  return seed_val = sd;
}

//static long get_seed(void) {
//  return seed_val;
//}

/* returns a random unsigned integer */
unsigned long int randlcg()
{
  if (seed_val <= quotient) {
    seed_val = (seed_val * 16807L) % LONG_MAX;
  } else {
    long int high_part = seed_val / quotient;
    long int low_part  = seed_val % quotient;

    long int test = 16807L * low_part - my_remainder * high_part;

    if (test > 0) {
      seed_val = test;
    } else {
      seed_val = test + LONG_MAX;
    }

  }

  return seed_val;
}

#endif /* 0*/
