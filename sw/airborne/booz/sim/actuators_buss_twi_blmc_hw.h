#ifndef ACTUATORS_BUSS_TWI_BLMC_HW_H
#define ACTUATORS_BUSS_TWI_BLMC_HW_H

#include "airframe.h"
#include "booz2_supervision.h"

#define BUSS_TWI_BLMC_NB 4
extern uint8_t buss_twi_blmc_motor_power[BUSS_TWI_BLMC_NB];

#define Actuator(i) buss_twi_blmc_motor_power[i]
#define SetActuatorsFromCommands(_motors_on) {				\
    pprz_t mixed_commands[SERVOS_NB];					\
    BOOZ2_SUPERVISION_RUN(mixed_commands, booz2_commands, _motors_on);	\
    Actuator(SERVO_FRONT) = (uint8_t)mixed_commands[SERVO_FRONT];	\
    Actuator(SERVO_BACK)  = (uint8_t)mixed_commands[SERVO_BACK];	\
    Actuator(SERVO_RIGHT) = (uint8_t)mixed_commands[SERVO_RIGHT];	\
    Actuator(SERVO_LEFT)  = (uint8_t)mixed_commands[SERVO_LEFT];	\
  }

extern uint8_t twi_blmc_nb_err;

#endif /* ACTUATORS_BUSS_TWI_BLMC_HW_H */
