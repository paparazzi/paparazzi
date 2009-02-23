#ifndef BOOZ2_FILTER_ATTITUDE_H
#define BOOZ2_FILTER_ATTITUDE_H

#include "std.h"
#include "booz_geometry_int.h"
#include "pprz_algebra_int.h"

#define BOOZ2_AHRS_UNINIT  0
#define BOOZ2_AHRS_RUNNING 1

struct BoozAhrs {
  struct booz_ieuler ltp_to_body_euler;
  struct booz_iquat  ltp_to_body_quat;
  struct booz_ieuler ltp_to_imu_euler;
  struct booz_iquat  ltp_to_imu_quat;
  struct booz_ivect  imu_rate;
  struct booz_ivect  body_rate;  
  struct booz_iquat  body_to_imu_quat;
  uint8_t status;
};

extern struct BoozAhrs booz_ahrs;

extern uint8_t booz2_ahrs_status;

extern void booz2_ahrs_init(void);
extern void booz2_ahrs_align(void);
extern void booz2_ahrs_propagate(void);
extern void booz2_ahrs_update(void);


#endif /* BOOZ2_ATTITUDE_FILTER_H */
