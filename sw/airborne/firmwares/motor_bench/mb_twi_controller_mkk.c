#include "mb_twi_controller_mkk.h"

#include <string.h>

#include "i2c.h"

uint8_t mb_buss_twi_command;

uint8_t mb_buss_twi_nb_overun;
uint8_t mb_buss_twi_i2c_done;


#define MB_BUSS_TWI_CONTROLLER_MAX_CMD 200
/*
  Slave address
  front = 0x52
  back  = 0x54
  right = 0x56
  left  = 0x58
*/
#define MB_BUSS_TWI_CONTROLLER_ADDR 0x56

void mb_twi_controller_init(void)
{
  mb_buss_twi_nb_overun = 0;
  mb_buss_twi_i2c_done = TRUE;
}

void mb_twi_controller_set(float throttle)
{
  if (mb_buss_twi_i2c_done) {
    mb_buss_twi_command = throttle * MB_BUSS_TWI_CONTROLLER_MAX_CMD;
    i2c_buf[0] = mb_buss_twi_command;
    i2c_transmit(MB_BUSS_TWI_CONTROLLER_ADDR, 1, &mb_buss_twi_i2c_done);
  } else {
    mb_buss_twi_nb_overun++;
  }
}
