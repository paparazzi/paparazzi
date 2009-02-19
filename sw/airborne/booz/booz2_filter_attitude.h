#ifndef BOOZ2_FILTER_ATTITUDE_H
#define BOOZ2_FILTER_ATTITUDE_H

#include "std.h"
#include "booz_geometry_int.h"

#define BOOZ2_FILTER_ATTITUDE_UNINIT  0
#define BOOZ2_FILTER_ATTITUDE_RUNNING 1

struct Booz_ahrs_state {
  struct Pprz_int32_euler euler;
  struct Pprz_int32_rate  rate;
  uint8_t status;
};

extern struct Booz_ahrs_state booz_ahrs_state;

extern struct booz_ieuler booz2_filter_attitude_euler_aligned;
extern struct booz_iquat  booz2_filter_attitude_quat_aligned;


extern struct booz_ieuler booz2_filter_attitude_euler;
extern struct booz_ivect  booz2_filter_attitude_rate;


extern uint8_t booz2_filter_attitude_status;

extern void booz2_filter_attitude_init(void);
extern void booz2_filter_attitude_align(void);
extern void booz2_filter_attitude_propagate(void);
extern void booz2_filter_attitude_update(void);


#endif /* BOOZ2_ATTITUDE_FILTER_H */
