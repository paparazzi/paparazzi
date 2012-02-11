#ifndef SERVOS_CSC_H
#define SERVOS_CSC_H

#include "LPC21xx.h"
#include "generated/airframe.h"
#include "actuators.h"
#include "mcu_periph/sys_time.h"

#define SERVOS_TICS_OF_USEC(s) CPU_TICKS_OF_USEC(s)
#define ChopServo(x,a,b) Chop(x, a, b)
#define SERVO_COUNT 4

#define Actuator(i) actuators[i]

static inline void ActuatorsCommit(void)
{

}

#endif /* SERVOS_CSC_H */
