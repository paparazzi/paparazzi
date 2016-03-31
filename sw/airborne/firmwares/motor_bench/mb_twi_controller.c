#include "mb_twi_controller.h"

#include <string.h>

#include "i2c.h"

uint16_t mb_twi_command;

uint8_t mb_twi_nb_overun;
uint8_t mb_twi_i2c_done;


void mb_twi_controller_init(void)
{
  mb_twi_nb_overun = 0;
  mb_twi_i2c_done = true;
}

void mb_twi_controller_set(float throttle)
{

  LED_TOGGLE(1);

  if (mb_twi_i2c_done) {
    mb_twi_command = throttle * MB_TWI_CONTROLLER_MAX_CMD;
    i2c_buf[0] = (uint8_t)(mb_twi_command & 0xFF);
    i2c_buf[1] = (uint8_t)(mb_twi_command >> 8);
    i2c_transmit(MB_TWI_CONTROLLER_ADDR, 2, &mb_twi_i2c_done);
  } else {
    mb_twi_nb_overun++;
  }

}
