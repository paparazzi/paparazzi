#include "actuators.h"

uint8_t buss_twi_blmc_motor_power[BUSS_TWI_BLMC_NB];
volatile bool_t  buss_twi_blmc_status;
uint8_t buss_twi_blmc_nb_err;
volatile bool_t  buss_twi_blmc_i2c_done;
volatile uint8_t buss_twi_blmc_idx;

const uint8_t buss_twi_blmc_addr[BUSS_TWI_BLMC_NB] = { 0x52, 0x54, 0x56, 0x58 };

void actuators_init ( void ) {
  uint8_t i;
  for (i=0; i<BUSS_TWI_BLMC_NB;i++)
    buss_twi_blmc_motor_power[i] = 0;
  buss_twi_blmc_status = BUSS_TWI_BLMC_STATUS_IDLE;
  buss_twi_blmc_nb_err = 0;
  buss_twi_blmc_i2c_done = TRUE;
}
