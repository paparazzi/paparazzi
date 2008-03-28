#include "vor_lf_filter.h"

#include <stdlib.h>

void vor_lf_filter_init(struct Filter* f, unsigned nb_num, unsigned nb_den, 
			const FLOAT_T* num, const FLOAT_T* den) {
  int i;

  f->nb_num = nb_num;
  f->num = num;
  f->x = malloc(f->nb_num * sizeof(FLOAT_T));
  for (i=0; i<f->nb_num; i++) {
    f->x[i] = 0.;
  }

  f->nb_den = nb_den;
  f->den = den;
  f->y = malloc(f->nb_den * sizeof(FLOAT_T));
  for (i=0; i<f->nb_den; i++) {
    f->y[i] = 0.;
  }

}


FLOAT_T vor_lf_filter_run(struct Filter *f, FLOAT_T xn) {
  FLOAT_T yn = 0;
  int i;
  f->x[0] = xn;
  // filter equation
  for (i=0; i<f->nb_num; i++)
    yn += f->num[i] * f->x[i];
  for (i=1 ; i< f->nb_den; i++)
    yn -= f->den[i] * f->y[i];
  // shifting
  for (i=f->nb_num-1; i>0 ; i--)
    f->x[i] = f->x[i-1];
  for (i=f->nb_den-1; i>0 ; i--)
    f->y[i] = f->y[i-1];
  f->y[1] = yn;
  return yn;
}
