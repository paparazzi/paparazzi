#include "booz_ami601.h"

uint8_t ami601_foo1;
uint8_t ami601_foo2;
uint8_t ami601_foo3;
uint16_t ami601_values[AMI601_NB_CHAN];

volatile uint8_t ami601_status;
volatile bool_t ami601_i2c_done;
volatile uint32_t ami601_nb_err;

void ami601_init( void ) {

  uint8_t i;
  for (i=0; i< AMI601_NB_CHAN; i++) {
    ami601_values[i] = 0;
  }
  ami601_i2c_done = TRUE;
  ami601_nb_err = 0;
  ami601_status = AMI601_IDLE;


}

void ami601_read( void ) {
  if (ami601_status != AMI601_IDLE) {
    ami601_nb_err++;
  }
  else {
    ami601_i2c_done = FALSE;
    ami601_status = AMI601_SENDING_REQ;
    i2c1_buf[0] = 0x55;
    i2c1_buf[1] = 0xAA;
    i2c1_buf[2] = 0x14;
    i2c1_transmit(AMI601_SLAVE_ADDR, 3, &ami601_i2c_done);
  }
}
