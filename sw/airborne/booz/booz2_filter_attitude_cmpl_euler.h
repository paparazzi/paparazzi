#ifndef BOOZ2_FILTER_ATTITUDE_CMPL_EULER_H
#define BOOZ2_FILTER_ATTITUDE_CMPL_EULER_H

#include "booz_ahrs.h"
#include "std.h"
#include "pprz_algebra_int.h"

extern struct Int32Rates  booz2_face_gyro_bias;
extern struct Int32Eulers booz2_face_measure;
extern struct Int32Eulers booz2_face_residual;
extern struct Int32Eulers booz2_face_uncorrected;
extern struct Int32Eulers booz2_face_corrected;

extern int32_t booz2_face_reinj_1;


#endif /* BOOZ2_FILTER_ATTITUDE_CMPL_EULER_H */
