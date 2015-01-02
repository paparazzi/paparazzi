#ifndef SERVOS_4015_HW_H
#define SERVOS_4015_HW_H

#include <inttypes.h>
#include "std.h"

#include "LPC21xx.h"
#include "mcu_periph/sys_time.h"

#include BOARD_CONFIG

/* PWM prescaler, set PWM input clock to 15MHz, PWM_CLK = PCLK / PWM_PRESCALER */

#if (PCLK == 15000000)
#define PWM_PRESCALER   1
#else

#if (PCLK == 30000000)
#define PWM_PRESCALER   2
#else

#if (PCLK == 60000000)
#define PWM_PRESCALER   4
#else

#error unknown PCLK frequency
#endif
#endif
#endif

#define PWM_TICS_OF_USEC(us)   (uint32_t)((us) *1e-6 * PCLK / PWM_PRESCALER + 0.5)

#define SERVOS_TICS_OF_USEC(s) PWM_TICS_OF_USEC(s)
#define ChopServo(x,a,b) Chop(x, a, b)

#define _4015_NB_CHANNELS 8
extern uint16_t servos_values[_4015_NB_CHANNELS];
#define Actuator4015Set(_i, _v) { servos_values[_i] = SERVOS_TICS_OF_USEC(_v); }

extern void actuators_4015_init(void);
#define Actuators4015Commit() {}
#define Actuators4015Init() actuators_4015_init()

void PWM_ISR(void) __attribute__((naked));


#endif /* SERVOS_4015_HW_H */
