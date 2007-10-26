#ifndef ACTUATORS_BUSS_TWI_BLMC_HW_H
#define ACTUATORS_BUSS_TWI_BLMC_HW_H

#include "std.h"

extern volatile bool_t  buss_twi_blmc_status;
extern volatile uint8_t buss_twi_blmc_nb_err;

#define SERVOS_TICS_OF_USEC(s) SYS_TICS_OF_USEC(s)
#define ChopServo(x,a,b) Chop(x, a, b)
#define Actuator(i) actuators[i]
#define ActuatorsCommit() {}

#endif /* ACTUATORS_BUSS_TWI_BLMC_HW_H */
