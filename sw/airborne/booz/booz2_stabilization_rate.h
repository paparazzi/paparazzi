#ifndef BOOZ2_STABILIZATION_RATE
#define BOOZ2_STABILIZATION_RATE

#include "pprz_algebra_int.h"

extern void booz2_stabilization_rate_init(void);
extern void booz2_stabilization_rate_read_rc(void);
extern void booz2_stabilization_rate_run(void);

extern struct Int32Rates booz2_stabilization_rate_measure;
extern struct Int32Rates booz2_stabilization_rate_sp;
extern struct Int32Rates booz2_stabilization_rate_gain;

#endif /* BOOZ2_STABILIZATION_RATE */
