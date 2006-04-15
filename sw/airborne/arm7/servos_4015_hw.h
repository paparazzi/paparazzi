#ifndef SERVOS_4015_HW_H
#define SERVOS_4015_HW_H

#include <inttypes.h>
#include "std.h"

#include "LPC21xx.h"
#include "sys_time.h"

#include CONFIG

#define SERVOS_TICS_OF_USEC(s) SYS_TICS_OF_USEC(s)
#define ChopServo(x,a,b) Chop(x, a, b)

#define _4015_NB_CHANNELS 8
extern uint16_t servos_values[_4015_NB_CHANNELS];
#define Actuator(i) servos_values[i]

void PWM_ISR ( void ) __attribute__((naked));


#endif /* SERVOS_4015_HW_H */
