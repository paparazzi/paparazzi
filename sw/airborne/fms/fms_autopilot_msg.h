#ifndef FMS_AUTOPILOT_H
#define FMS_AUTOPILOT_H

#include <inttypes.h>
#include "math/pprz_algebra_int.h"

struct AutopilotMessageFoo {
  uint8_t foo;
  uint8_t bar;
  uint8_t blaa;
};


struct AutopilotMessageBethUp {
  struct Int16Vect3 gyro;
  struct Int16Vect3 accel;
  struct Int16Vect3 bench_sensor;

};

struct AutopilotMessageBethDown {
  uint8_t motor_front;
  uint8_t motor_back;
};

union AutopilotMessageBeth {
  struct AutopilotMessageBethUp up;
  struct AutopilotMessageBethDown down;
};


#define TW_BUF_LEN 63
struct AutopilotMessageTWUp {
  uint8_t tw_len;
  uint8_t data[TW_BUF_LEN];
};

struct AutopilotMessageTWDown {
  uint8_t tw_len;
  uint8_t data[TW_BUF_LEN];
};

union AutopilotMessageTW {
  struct AutopilotMessageTWUp up;
  struct AutopilotMessageTWDown down;
};


#endif /* FMS_AUTOPILOT_H */
