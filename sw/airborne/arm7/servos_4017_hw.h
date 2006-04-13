#ifndef SERVOS_4017_HW_H
#define SERVOS_4017_HW_H

#include <inttypes.h>

#define SERVOS_TICS_OF_USEC(s) SYS_TICS_OF_USEC(s)
#define ChopServo(x,a,b) Chop(x, a, b)

#define _4017_NB_CHANNELS 10
extern uint16_t servos_values[_4017_NB_CHANNELS];
#define Actuator(i) servos_values[i]


void PWM_ISR ( void ) __attribute__((naked));

static inline void actuators_init ( void ) {


}

#endif /* SERVOS_4017_HW_H */
