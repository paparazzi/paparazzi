#ifndef ACTUATORS_BUSS_TWI_BLMC_HW_H
#define ACTUATORS_BUSS_TWI_BLMC_HW_H

#include "std.h"

extern volatile bool_t  buss_twi_blmc_status;
extern volatile uint8_t buss_twi_blmc_nb_err;

#define SERVOS_TICS_OF_USEC(s) SYS_TICS_OF_USEC(s)
#define ChopServo(x,a,b) ((x)>(b)?(b):(x))
#define Actuator(i) buss_twi_blmc_motor_power[i]
#define ActuatorsCommit() {}

#define BUSS_TWI_BLMC_NB 4
extern uint8_t buss_twi_blmc_motor_power[BUSS_TWI_BLMC_NB];

#endif /* ACTUATORS_BUSS_TWI_BLMC_HW_H */
