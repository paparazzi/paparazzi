#ifndef ACTUATORS_DRIVER_H
#define ACTUATORS_DRIVER_H

#include "std.h"

typedef void (*act_set_t)(uint8_t channel, int16_t value);
typedef void (*act_commit_t)(void);

struct actuator_driver {
  act_set_t set;
  act_commit_t commit;
};

/** Register before using this driver. Both callbacks are required. */
extern void actuators_register_driver(uint8_t driver_id, struct actuator_driver driver);

static inline void actuators_noop_set(uint8_t channel, int16_t value)
{
  (void)channel;
  (void)value;
}

static inline void actuators_noop_commit(void) {}

#endif
