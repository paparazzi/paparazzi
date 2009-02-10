#ifndef BOOZ2_STABILIZATION_RATE
#define BOOZ2_STABILIZATION_RATE

#include "booz_geometry_int.h"

extern void booz2_stabilization_rate_init(void);
extern void booz2_stabilization_rate_read_rc(void);
extern void booz2_stabilization_rate_run(void);

extern struct booz_ivect booz2_stabilization_rate_measure;
extern struct booz_ivect booz2_stabilization_rate_sp;
extern struct booz_ivect booz2_stabilization_rate_gain;

#endif /* BOOZ2_STABILIZATION_RATE */
