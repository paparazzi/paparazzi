#ifndef OVERO_TEST_PASSTHROUGH_H
#define OVERO_TEST_PASSTHROUGH_H

#include "std.h"
#include "booz/booz_imu.h"

struct OveroTestPassthrough {
  struct BoozImuFloat imu;
  uint32_t io_proc_msg_cnt;
  uint32_t io_proc_err_cnt;
  uint16_t servos_outputs_usecs[6]; /* FIXME */
};

extern struct OveroTestPassthrough otp;

#endif /* OVERO_TEST_PASSTHROUGH_H */
