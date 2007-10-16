#include "mb_twi_controller.h"

#include <string.h>

#include "i2c.h"

uint16_t mb_twi_command;

uint8_t mb_twi_nb_overun;
uint8_t mb_twi_i2c_done;


void mb_twi_controller_init(void) {
  mb_twi_nb_overun = 0;
  mb_twi_i2c_done = TRUE;

}

void mb_twi_controller_set( float throttle ) {
  if (mb_twi_i2c_done) {
    mb_twi_command = throttle * MB_TWI_CONTROLLER_MAX_CMD;
    uint8_t lb = (uint8_t)(mb_twi_command&0xFF);
    uint8_t hb = (uint8_t)(mb_twi_command>>8);
    const uint8_t msg[] = { MB_TWI_CONTROLLER_ADDR, hb, lb};
    memcpy((void*)i2c_buf, msg, sizeof(msg));
    i2c_transmit(MB_TWI_CONTROLLER_ADDR, sizeof(msg), &mb_twi_i2c_done);
  }
  else
    mb_twi_nb_overun++;

}
