#include "nps_random.h"


#include <math.h>

void double_vect3_add_gaussian_noise(struct DoubleVect3* vect, struct DoubleVect3* std_dev) {
  vect->x += get_gaussian_noise() * std_dev->x;
  vect->y += get_gaussian_noise() * std_dev->y;
  vect->z += get_gaussian_noise() * std_dev->z;
}

/* 
   http://www.taygeta.com/random/gaussian.html 
*/

#include "booz_r250.h"

double get_gaussian_noise(void) {
 
  double x1;
  static int nb_call = 0;
  static double x2, w;
  if (nb_call==0) r250_init(0);
  nb_call++;
  if (nb_call%2) {
    do {
      x1 = 2.0 * dr250() - 1.0;
      x2 = 2.0 * dr250() - 1.0;
      w = x1 * x1 + x2 * x2;
    } while ( w >= 1.0 );
    
    w = sqrt( (-2.0 * log( w ) ) / w );
    return x1 * w;
  }
  else
    return x2 * w;
}
