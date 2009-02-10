#ifndef BOOZ2_FILTER_ATTITUDE_CMPL_EULER_H
#define BOOZ2_FILTER_ATTITUDE_CMPL_EULER_H

#include "booz2_filter_attitude.h"
#include "std.h"
#include "booz_geometry_int.h"

extern struct booz_ivect  booz2_face_gyro_bias;
extern struct booz_ieuler booz2_face_measure;
extern struct booz_ieuler booz2_face_residual;
extern struct booz_ieuler booz2_face_uncorrected;
extern struct booz_ieuler booz2_face_corrected;

extern int32_t booz2_face_reinj_1;


#endif /* BOOZ2_FILTER_ATTITUDE_CMPL_EULER_H */
