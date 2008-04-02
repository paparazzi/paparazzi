#include "mb_twi_controller.h"

#include "i2c.h"

#include "led.h"

uint8_t mb_twi_command;

uint8_t mb_twi_nb_overun;
uint8_t mb_twi_i2c_done;


#define MB_TWI_CONTROLLER_MAX_CMD 200
#define MB_TWI_CONTROLLER_ADDR 0x02

void mb_twi_controller_init(void) {
  mb_twi_nb_overun = 0;
  mb_twi_i2c_done = TRUE;
}

void mb_twi_controller_set( float throttle ) {
  if (mb_twi_i2c_done) {
    LED_TOGGLE(1);
    uint8_t pitch = 100;
    uint8_t roll  = 100;
    uint8_t yaw   = 100;
    mb_twi_command = throttle * MB_TWI_CONTROLLER_MAX_CMD;
    i2c_buf[0] = pitch;
    i2c_buf[1] = roll;
    i2c_buf[2] = yaw;
    i2c_buf[3] = mb_twi_command;
    i2c_transmit(MB_TWI_CONTROLLER_ADDR, 4, &mb_twi_i2c_done);
  }
  else
    mb_twi_nb_overun++;
}
