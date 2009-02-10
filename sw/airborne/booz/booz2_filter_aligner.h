#ifndef BOOZ2_FILTER_ALIGNER_H
#define BOOZ2_FILTER_ALIGNER_H

#include "std.h"
#include "booz_geometry_int.h"

#define BOOZ2_FILTER_ALIGNER_UNINIT  0
#define BOOZ2_FILTER_ALIGNER_RUNNING 1
#define BOOZ2_FILTER_ALIGNER_LOCKED  2


extern uint8_t booz2_filter_aligner_status;

extern struct booz_ivect booz2_filter_aligner_lp_gyro;
extern struct booz_ivect booz2_filter_aligner_lp_accel;
extern struct booz_ivect booz2_filter_aligner_lp_mag;
extern int32_t           booz2_filter_aligner_noise;
extern int32_t           booz2_filter_aligner_low_noise_cnt;

extern void booz2_filter_aligner_init(void);

extern void booz2_filter_aligner_run(void);

#endif /* BOOZ2_FILTER_ALIGNER_H */
