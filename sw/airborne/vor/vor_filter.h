#ifndef VOR_FILTER_H
#define VOR_FILTER_H

#include <inttypes.h>

#define VOR_FILTER_MAX_SIZE 7

struct VorFilter {
  unsigned nb_num;
  unsigned nb_den;
  const FLOAT_T* num;
  const FLOAT_T* den;
  FLOAT_T x[VOR_FILTER_MAX_SIZE];
  FLOAT_T y[VOR_FILTER_MAX_SIZE];
};

extern void vor_filter_init(struct VorFilter* f,
			    unsigned nb_num, unsigned nb_den,
			    const FLOAT_T* num, const FLOAT_T* den);

extern FLOAT_T vor_filter_run(struct VorFilter *f, FLOAT_T xn);


#endif /* VOR_FILTER_H */

