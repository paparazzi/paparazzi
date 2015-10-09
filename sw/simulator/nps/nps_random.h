#ifndef NPS_RANDOM_H
#define NPS_RANDOM_H

#include "math/pprz_algebra_double.h"

extern double get_gaussian_noise(void);
extern void double_vect3_add_gaussian_noise(struct DoubleVect3 *vect, struct DoubleVect3 *std_dev);
extern void double_vect3_get_gaussian_noise(struct DoubleVect3 *vect, struct DoubleVect3 *std_dev);
extern void double_vect3_update_random_walk(struct DoubleVect3 *rw, struct DoubleVect3 *std_dev, double dt,
    double thau);

extern void float_vect3_add_gaussian_noise(struct FloatVect3 *vect, struct FloatVect3 *std_dev);
extern void float_rates_add_gaussian_noise(struct FloatRates *vect, struct FloatRates *std_dev);



#endif /* NPS_RANDOM_H */

