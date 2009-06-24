#ifndef NPS_RANDOM_H
#define NPS_RANDOM_H

#include "pprz_algebra_double.h"

extern double get_gaussian_noise(void);
extern void double_vect3_add_gaussian_noise(struct DoubleVect3* vect, struct DoubleVect3* std_dev);



#endif /* NPS_RANDOM_H */

