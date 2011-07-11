#ifndef SERVOS_DIRECT_HW_H
#define SERVOS_DIRECT_HW_H


#include "std.h"

#define ACTUATORS_PWM_NB 6
extern int32_t actuators_pwm_values[ACTUATORS_PWM_NB];

#include "actuators_pwm_arch.h"

#define SERVOS_TICS_OF_USEC(_v) (_v)
#define Actuator(_x)  actuators_pwm_values[_x]
//#define ChopServo(x,a,b) Chop(x, a, b)
#define ActuatorsCommit  actuators_pwm_commit


#endif /* SERVOS_DIRECT_HW_H */
