#ifndef SERVOS_DIRECT_HW_H
#define SERVOS_DIRECT_HW_H


#include "std.h"

#define BOOZ_ACTUATORS_PWM_NB 6
extern int32_t booz_actuators_pwm_values[BOOZ_ACTUATORS_PWM_NB];

#include "actuators/booz_actuators_pwm_arch.h"

#define SERVOS_TICS_OF_USEC(_v) (_v)
#define Actuator(_x)  booz_actuators_pwm_values[_x]
#define ChopServo(x,a,b) Chop(x, a, b)
#define ActuatorsCommit  booz_actuators_pwm_commit


#endif /* SERVOS_DIRECT_HW_H */
