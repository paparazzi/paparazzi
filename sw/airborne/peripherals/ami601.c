#include "peripherals/ami601.h"

uint8_t ami601_foo1;
uint8_t ami601_foo2;
uint8_t ami601_foo3;
uint16_t ami601_values[AMI601_NB_CHAN];

volatile uint8_t ami601_status;
struct i2c_transaction  ami601_i2c_trans;
volatile uint32_t ami601_nb_err;

void ami601_init(void)
{

  uint8_t i;
  for (i = 0; i < AMI601_NB_CHAN; i++) {
    ami601_values[i] = 0;
  }
  ami601_i2c_trans.status = I2CTransSuccess;
  ami601_i2c_trans.slave_addr = AMI601_SLAVE_ADDR;
  ami601_nb_err = 0;
  ami601_status = AMI601_IDLE;

}

void ami601_read(void)
{
  if (ami601_status != AMI601_IDLE) {
    ami601_nb_err++;
    ami601_status = AMI601_IDLE;
  } else {
    ami601_status = AMI601_SENDING_REQ;
    ami601_i2c_trans.type = I2CTransTx;
    ami601_i2c_trans.len_w = 3;
    ami601_i2c_trans.buf[0] = 0x55;
    ami601_i2c_trans.buf[1] = 0xAA;
    ami601_i2c_trans.buf[2] = 0x14;
    i2c_submit(&i2c1, &ami601_i2c_trans);
  }
}
