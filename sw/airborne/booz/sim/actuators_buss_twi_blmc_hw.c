#include "actuators.h"


uint16_t actuators[SERVOS_NB];

uint8_t twi_blmc_nb_err;
uint8_t buss_twi_blmc_motor_power[BUSS_TWI_BLMC_NB];

void actuators_init( void ) {}
