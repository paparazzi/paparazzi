#ifndef BOOZ_AHRS_ALIGNER_H
#define BOOZ_AHRS_ALIGNER_H

#include "std.h"
#include "pprz_algebra_int.h"

#define BOOZ_AHRS_ALIGNER_UNINIT  0
#define BOOZ_AHRS_ALIGNER_RUNNING 1
#define BOOZ_AHRS_ALIGNER_LOCKED  2

struct BoozAhrsAligner {
  struct Int32Rates lp_gyro;
  struct Int32Vect3 lp_accel;
  struct Int32Vect3 lp_mag;
  int32_t           noise;
  int32_t           low_noise_cnt;
  uint8_t           status;
};

extern struct BoozAhrsAligner booz_ahrs_aligner;

extern void booz_ahrs_aligner_init(void);
extern void booz_ahrs_aligner_run(void);

#endif /* BOOZ_AHRS_ALIGNER_H */
