#include "buss_twi_blmc_hw.h"
#include <inttypes.h>

// From sys_time_hw.h
#define T0_PCLK_DIV 1

// Number of PCLK cycles to wait between sending commands
#define TICS_OF_BLMC_IDX (I2C_CYCLES_PER_MSG * (I2C0_SCLL + I2C0_SCLH) / T0_PCLK_DIV)

// Number of I2C cycles to wait before timeout and send next command (including some margin)
#define I2C_CYCLES_PER_MSG 48

uint8_t twi_blmc_nb_err;

uint8_t motor_power[BUSS_TWI_BLMC_NB];
static volatile bool_t  buss_twi_blmc_status;
static volatile bool_t  buss_twi_blmc_i2c_done;
static volatile uint8_t buss_twi_blmc_idx;
static uint32_t commit_start_time;

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

void motors_commit_next( void )
{
    buss_twi_blmc_idx++;				
    if (buss_twi_blmc_idx < BUSS_TWI_BLMC_NB_SEND)
      buss_twi_blmc_send_next();
    else						
      buss_twi_blmc_status = BUSS_TWI_BLMC_STATUS_IDLE;	
}

void motors_commit(int force)
{

  if (force || buss_twi_blmc_status == BUSS_TWI_BLMC_STATUS_IDLE) {
    I2c0SendStop();
    buss_twi_blmc_idx = 0;						
    buss_twi_blmc_status = BUSS_TWI_BLMC_STATUS_BUSY;			
    buss_twi_blmc_send_next();

    commit_start_time = T0TC;

  }
}


void motors_callback_nop()
{
  // do nothing...
  // We don't use this to kick the next command automatically anymore
}

void motors_event( void )
{
  // if busy sending, check progress
  if (buss_twi_blmc_status == BUSS_TWI_BLMC_STATUS_BUSY) {
    // check timer 0 counter to see if it is time for next message
    if ((T0TC - commit_start_time) > TICS_OF_BLMC_IDX) {
      // abort any previous activity
      I2c0SendStop();
      // advance index and start transmit on next message
      motors_commit_next();
      // record current time
      commit_start_time = T0TC;
    }
  }
}
