#ifndef SYS_TIME_HW_H
#define SYS_TIME_HW_H

#include <unistd.h>

#define SYS_TICS_OF_SEC(x) (x)
#define SYS_TICS_OF_USEC(x) (x)
#define SYS_TICS_OF_NSEC(x) (x)
#define SIGNED_SYS_TICS_OF_SEC(x) (x)
#define SIGNED_SYS_TICS_OF_USEC(x) (x)

#define SEC_OF_SYS_TICS(st) (st)
#define MSEC_OF_SYS_TICS(st) (st)
#define USEC_OF_SYS_TICS(st) (st)

#define SysTimeChronoStart() { }
#define SysTimeChronoStop() { }
#define SysTimeChronoStartDisableIRQ() { }
#define SysTimeChronoStopEnableIRQAndSendUS(_tag) { }

#define SysTimeTimerStart(_t) { }
#define SysTimeTimer(_t) (_t)
#define SysTimeTimerStop(_t) { }

static inline void sys_time_init( void ) {}

#define sys_time_periodic() TRUE
#define sys_time_usleep(x) usleep(x)

#endif /* SYS_TIME_HW_H */
