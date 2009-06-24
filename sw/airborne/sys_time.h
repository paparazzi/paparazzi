#ifndef SYS_TIME_H
#define SYS_TIME_H

#include <inttypes.h>
#include CONFIG

extern uint16_t cpu_time_sec;

#ifndef READYBOARD_SYS_TIME
#include "sys_time_hw.h"
#endif

#endif /* SYS_TIME_H */
