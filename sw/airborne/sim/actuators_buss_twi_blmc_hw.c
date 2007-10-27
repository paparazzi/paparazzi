#include "actuators.h"

volatile bool_t  buss_twi_blmc_status;
volatile uint8_t buss_twi_blmc_nb_err;

uint8_t buss_twi_blmc_motor_power[BUSS_TWI_BLMC_NB];

void actuators_init ( void ) {}
