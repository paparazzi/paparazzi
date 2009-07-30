#include "buss_twi_blmc_hw.h"
#include <inttypes.h>

uint8_t twi_blmc_nb_err;

uint8_t motor_power[BUSS_TWI_BLMC_NB];
static volatile bool_t  buss_twi_blmc_status;
static volatile bool_t  buss_twi_blmc_i2c_done;
static volatile uint8_t buss_twi_blmc_idx;

const uint8_t buss_twi_blmc_addr[BUSS_TWI_BLMC_NB] = BUSS_BLMC_ADDR;

void motors_init ( void ) {
  uint8_t i;
  for (i=0; i<BUSS_TWI_BLMC_NB;i++)
    motor_power[i] = 0;
  buss_twi_blmc_status = BUSS_TWI_BLMC_STATUS_IDLE;
  twi_blmc_nb_err = 0;
  buss_twi_blmc_i2c_done = TRUE;
}

void motors_set_motor(uint8_t id, int16_t value)
{
  // insert range checks
  if(value < 0)
    motor_power[id] = 0;
  else if(value > 255)
    motor_power[id] = 255;
  else
    motor_power[id] = value;
}

static void buss_twi_blmc_send_next()
{
  i2c0_buf[0] = motor_power[buss_twi_blmc_idx];		             
  i2c0_transmit(buss_twi_blmc_addr[buss_twi_blmc_idx], 1, &buss_twi_blmc_i2c_done); 
}


void motors_commit()
{
  buss_twi_blmc_idx = 0;						
  buss_twi_blmc_status = BUSS_TWI_BLMC_STATUS_BUSY;			
  buss_twi_blmc_send_next();
}


void motors_commit_next()
{
    buss_twi_blmc_idx++;				
    if (buss_twi_blmc_idx < BUSS_TWI_BLMC_NB)
      buss_twi_blmc_send_next();
    else						
      buss_twi_blmc_status = BUSS_TWI_BLMC_STATUS_IDLE;	
}






