#ifndef BOOZ_FMS_TEST_SIGNAL_H
#define BOOZ_FMS_TEST_SIGNAL_H

#include "std.h"

#define BOOZ_FMS_TEST_SIGNAL_MODE_DISABLED 0
#define BOOZ_FMS_TEST_SIGNAL_MODE_ATTITUDE 1
#define BOOZ_FMS_TEST_SIGNAL_MODE_VERTICAL 2

extern uint8_t  booz_fms_test_signal_mode;

extern uint32_t booz_fms_test_signal_period;
extern uint32_t booz_fms_test_signal_amplitude;
extern uint8_t  booz_fms_test_signal_axe;
extern uint32_t booz_fms_test_signal_counter;

extern void booz_fms_impl_init(void);
extern void booz_fms_impl_periodic(void);

#define booz2_fms_test_signal_SetPeriod(_val) { \
    booz_fms_test_signal_period = _val;		\
    booz_fms_test_signal_counter = 0;		\
}

#define booz2_fms_test_signal_SetTsMode(_val) {				\
    booz_fms_test_signal_mode = _val;					\
    if (booz_fms_test_signal_mode == BOOZ_FMS_TEST_SIGNAL_MODE_VERTICAL) \
      booz_fms_input.v_mode = BOOZ2_GUIDANCE_V_MODE_HOVER;		\
    else								\
      booz_fms_input.v_mode = BOOZ2_GUIDANCE_V_MODE_DIRECT;		\
}

#endif /* BOOZ_FMS_TEST_SIGNAL_H */

