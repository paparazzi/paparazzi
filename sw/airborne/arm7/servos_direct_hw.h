#ifndef SERVOS_DIRECT_HW_H
#define SERVOS_DIRECT_HW_H

#include "LPC21xx.h"

#define SERVOS_TICS_OF_USECS(s) SYS_TICS_OF_USEC(s)

#define SERVO_REG_0 PWMMR1
#define SERVO_REG_1 PWMMR2
#define SERVO_REG_2 PWMMR3
#define SERVO_REG_3 PWMMR4
#define SERVO_REG_4 PWMMR5
#define SERVO_REG_5 PWMMR6

#define COMMAND_(i) SERVO_REG_ ## i
#define Actuator(i) COMMAND_(i)

#endif /* SERVOS_DIRECT_HW_H */
