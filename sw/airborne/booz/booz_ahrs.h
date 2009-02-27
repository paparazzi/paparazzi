#ifndef BOOZ_AHRS_H
#define BOOZ_AHRS_H

#include "std.h"
#include "pprz_algebra_int.h"

#define BOOZ_AHRS_UNINIT  0
#define BOOZ_AHRS_RUNNING 1

struct BoozAhrs {
  struct Int32Eulers ltp_to_body_euler;
  struct Int32Quat   ltp_to_body_quat;
  struct Int32RMat   ltp_to_body_rmat;
  struct Int32Eulers ltp_to_imu_euler;
  struct Int32Quat   ltp_to_imu_quat;
  struct Int32RMat   ltp_to_imu_rmat;
  struct Int32Rates  imu_rate;
  struct Int32Rates  body_rate;  
  uint8_t status;
};

extern struct BoozAhrs booz_ahrs;

extern void booz_ahrs_init(void);
extern void booz_ahrs_align(void);
extern void booz_ahrs_propagate(void);
extern void booz_ahrs_update(void);


#endif /* BOOZ_AHRS_H */
