#ifndef SYS_TIME_HW_H
#define SYS_TIME_HW_H

#include <unistd.h>

#define CPU_TICKS_OF_SEC(x) (x)
#define SIGNED_CPU_TICKS_OF_SEC(x) (x)

#define SEC_OF_CPU_TICKS(st) (st)
#define MSEC_OF_CPU_TICKS(st) (st)
#define USEC_OF_CPU_TICKS(st) (st)

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
