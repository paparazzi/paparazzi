#ifndef TL_AUTOPILOT_H
#define TL_AUTOPILOT_H

#include "std.h"

#define TL_AP_MODE_FAILSAFE 0
#define TL_AP_MODE_KILL     1
#define TL_AP_MODE_RATE     2
#define TL_AP_MODE_ATTITUDE 3
#define TL_AP_MODE_NAV      4

extern uint8_t tl_autopilot_mode;

extern void tl_autopilot_init(void);
extern void tl_autopilot_periodic_task(void);
extern void tl_autopilot_on_rc_event(void);

#define TRESHOLD_RATE_PPRZ (MIN_PPRZ / 2)
#define TRESHOLD_ATTITUDE_PPRZ  (MAX_PPRZ/2)

#if 0
#define TL_AP_MODE_OF_PPRZ(mode)				\
  ((mode) < TRESHOLD_RATE_PPRZ ? TL_AP_MODE_RATE :		\
   (mode) < TRESHOLD_ATTITUDE_PPRZ ? TL_AP_MODE_ATTITUDE :	\
   TL_AP_MODE_NAV )

#define TL_AP_MODE_OF_PPRZ(mode)				\
  ((mode) < TRESHOLD_RATE_PPRZ ? TL_AP_MODE_RATE :		\
   TL_AP_MODE_ATTITUDE )

#endif // 0

#define TL_AP_MODE_OF_PPRZ(mode)				\
  ((mode) < TRESHOLD_RATE_PPRZ ? TL_AP_MODE_ATTITUDE :		\
   TL_AP_MODE_NAV )




#endif /* TL_AUTOPILOT_H */
