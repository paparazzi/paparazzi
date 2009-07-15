#include <stdio.h> /* For NULL */
#include "i2c.h"

#define I2C_RECEIVE     1

#ifdef USE_I2C0

volatile uint8_t i2c0_status;
volatile uint8_t i2c0_buf[I2C0_BUF_LEN];
volatile uint8_t i2c0_len;
volatile uint8_t i2c0_index;
volatile uint8_t i2c0_slave_addr;

volatile bool_t* i2c0_finished;

void i2c0_init(void) {
  i2c0_status = I2C_IDLE;
  i2c0_hw_init();
  i2c0_finished = NULL;
}


void i2c0_receive(uint8_t slave_addr, uint8_t len, volatile bool_t* finished) {
  i2c0_len = len;
  i2c0_slave_addr = slave_addr | I2C_RECEIVE;
  i2c0_finished = finished;
  i2c0_status = I2C_BUSY;
  I2c0SendStart();
}

void i2c0_transmit(uint8_t slave_addr, uint8_t len, volatile bool_t* finished) {
  i2c0_len = len;
  i2c0_slave_addr = slave_addr & ~I2C_RECEIVE;
  i2c0_finished = finished;
  i2c0_status = I2C_BUSY;
  I2c0SendStart();
}

#endif /* USE_I2C0 */

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
