/*
 * actuators_pwm_arch.h
 *
 *  Created on: Nov 19, 2012
 *      Author: dhensen
 */

#ifndef ACTUATORS_PWM_ARCH_H_
#define ACTUATORS_PWM_ARCH_H_

#define SERVOS_TICS_OF_USEC(_v) (_v)

#define ActuatorPwmSet(_i, _v) {}
#define ActuatorsPwmCommit() {}

//extern void actuators_pwm_arch_init(void);

#endif /* ACTUATORS_PWM_ARCH_H_ */
