#include <stdio.h> /* For NULL */
#include "i2c.h"

#define I2C_RECEIVE     1

volatile uint8_t i2c_status;
volatile uint8_t i2c_buf[I2C_BUF_LEN];
volatile uint8_t i2c_len;
volatile uint8_t i2c_index;
volatile uint8_t i2c_slave_addr;

volatile bool_t* i2c_finished;

void i2c_init(void) {
  i2c_status = I2C_IDLE;
  i2c_hw_init();
  i2c_finished = NULL;
}


void i2c_receive(uint8_t slave_addr, uint8_t len, bool_t* finished) {
  i2c_len = len;
  i2c_slave_addr = slave_addr | I2C_RECEIVE;
  i2c_finished = finished;
  i2c_status = I2C_BUSY;
  I2cSendStart();
}

void i2c_transmit(uint8_t slave_addr, uint8_t len, bool_t* finished) {
  i2c_len = len;
  i2c_slave_addr = slave_addr & ~I2C_RECEIVE;
  i2c_finished = finished;
  i2c_status = I2C_BUSY;
  I2cSendStart();
}

