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
  i2c0_hw_init();
  i2c_finished = NULL;
}


void i2c_receive(uint8_t slave_addr, uint8_t len, volatile bool_t* finished) {
  i2c_len = len;
  i2c_slave_addr = slave_addr | I2C_RECEIVE;
  i2c_finished = finished;
  i2c_status = I2C_BUSY;
  I2cSendStart();
}

void i2c_transmit(uint8_t slave_addr, uint8_t len, volatile bool_t* finished) {
  i2c_len = len;
  i2c_slave_addr = slave_addr & ~I2C_RECEIVE;
  i2c_finished = finished;
  i2c_status = I2C_BUSY;
  I2cSendStart();
}



#ifdef USE_I2C1

volatile uint8_t i2c1_status;
volatile uint8_t i2c1_buf[I2C1_BUF_LEN];
volatile uint8_t i2c1_len;
volatile uint8_t i2c1_index;
volatile uint8_t i2c1_slave_addr;

volatile bool_t* i2c1_finished;

void i2c1_init(void) {
  i2c1_status = I2C_IDLE;
  i2c1_hw_init();
  i2c1_finished = NULL;
}


void i2c1_receive(uint8_t slave_addr, uint8_t len, volatile bool_t* finished) {
  i2c1_len = len;
  i2c1_slave_addr = slave_addr | I2C_RECEIVE;
  i2c1_finished = finished;
  i2c1_status = I2C_BUSY;
  I2c1SendStart();
}

void i2c1_transmit(uint8_t slave_addr, uint8_t len, volatile bool_t* finished) {
  i2c1_len = len;
  i2c1_slave_addr = slave_addr & ~I2C_RECEIVE;
  i2c1_finished = finished;
  i2c1_status = I2C_BUSY;
  I2c1SendStart();
}




#endif /* USE_I2C1 */
