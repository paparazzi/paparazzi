#ifndef BOOZ_DEBUG_H
#define BOOZ_DEBUG_H

#ifdef BOOZ_DEBUG

#include "uart.h"
#include "messages.h"
#include "downlink.h"

extern uint8_t booz_debug_mod;
extern uint8_t booz_debug_err;

#define DEBUG_IMU      0
#define DEBUG_MAX_1117 1
#define DEBUG_SCP1000  2


#define ASSERT(cond, mod, err) {					\
    if (!(cond)) {							\
      booz_debug_mod = mod;						\
      booz_debug_err = err;						\
      DOWNLINK_SEND_BOOZ_ERROR(&booz_debug_mod, &booz_debug_err);	\
    }									\
  }
#else
#define ASSERT(cond, mod, err) {}
#endif


#endif /* BOOZ_DEBUG_H */
