#ifndef BOOZ_AHRS_H
#define BOOZ_AHRS_H

#include "std.h"

#define BOOZ_AHRS_STATUS_UNINIT  0
#define BOOZ_AHRS_STATUS_RUNNING 1
#define BOOZ_AHRS_STATUS_CRASHED 2

extern uint8_t booz_ahrs_status;

extern float booz_ahrs_phi; 
extern float booz_ahrs_theta; 
extern float booz_ahrs_psi;

extern float booz_ahrs_p;
extern float booz_ahrs_q;
extern float booz_ahrs_r;

extern float booz_ahrs_bp;
extern float booz_ahrs_bq;
extern float booz_ahrs_br;

extern void booz_ahrs_init(void);
extern void booz_ahrs_start( const float* accel, const float* gyro, const int16_t* mag);
extern void booz_ahrs_run(const float* accel, const float* gyro, const int16_t* mag);
/*
  extern void booz_ahrs_predict( const float* gyro );
  extern void booz_ahrs_update( const float* accel, const int16_t* mag );
*/

//#define __AHRS_IMPL_HEADER(_typ, _ext) _t##_ext
//#define _AHRS_IMPL_HEADER(_typ, _ext) __AHRS_IMPL_HEADER(_typ, _ext)
//#define AHRS_IMPL_HEADER(_typ) _AHRS_IMPL_HEADER(_typ, ".h")
//#error BOOZ_AHRS_TYPE
//#include \"AHRS_IMPL_HEADER(BOOZ_AHRS_TYPE)\"

#define BOOZ_AHRS_MULTITILT  0
#define BOOZ_AHRS_EULER      1
#define BOOZ_AHRS_QUATERNION 2

#if defined BOOZ_AHRS_TYPE && BOOZ_AHRS_TYPE == BOOZ_AHRS_MULTITILT
#include "multitilt.h"
#elif defined BOOZ_AHRS_TYPE && BOOZ_AHRS_TYPE == BOOZ_AHRS_QUATERNION
#include "ahrs_quat_fast_ekf.h"
#endif


#endif /* BOOZ_AHRS_H */
