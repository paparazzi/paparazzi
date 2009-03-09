#ifndef BOOZ_AHRS_FLOAT_EKF_H
#define BOOZ_AHRS_FLOAT_EKF_H


#include "pprz_algebra_float.h"


extern void booz_ahrs_init(void);
extern void booz_ahrs_align(void);
extern void booz_ahrs_propagate(void);
extern void booz_ahrs_update(void);




#endif /* BOOZ_AHRS_FLOAT_EKF_H */

