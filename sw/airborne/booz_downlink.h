#ifndef BOOZ_DOWNLINK_H
#define BOOZ_DOWNLINK_H

#include "std.h"
#include "periodic.h"

static inline void booz_downlink_periodic_task(void) {
  PeriodicSendMain()
}


#endif /* BOOZ_DOWNLINK_H */
