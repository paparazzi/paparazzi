#include <stdio.h> /* For NULL */
#include "i2c.h"

#define I2C_RECEIVE     1

#ifdef USE_I2C0

volatile uint8_t i2c0_status;
volatile uint8_t i2c0_buf[I2C0_BUF_LEN];
volatile uint16_t i2c0_len_r;
volatile uint8_t  i2c0_len_w;
volatile uint8_t i2c0_index;
volatile uint8_t i2c0_slave_addr;
volatile bool_t  i2c0_stop_after_transmit;
volatile uint8_t i2c0_trx;

volatile bool_t* i2c0_finished;

void i2c0_init(void) {
  i2c0_status = I2C_IDLE;
  i2c0_hw_init();
  i2c0_finished = NULL;
}


void i2c0_receive(uint8_t slave_addr, uint16_t len, volatile bool_t* finished) {
  i2c0_trx = 0;
  i2c0_len_r = len;
  i2c0_slave_addr = slave_addr | I2C_RECEIVE;
  i2c0_finished = finished;
  i2c0_status = I2C_BUSY;
  i2c0_stop_after_transmit = TRUE; /** Default "historic" behaviour */
  I2c0SendStart();
}

static inline void i2c0_init_transmit(uint8_t slave_addr, uint8_t len, volatile bool_t* finished) {
  i2c0_trx = 0;
  i2c0_len_w = len;
  i2c0_slave_addr = slave_addr & ~I2C_RECEIVE;
  i2c0_finished = finished;
  i2c0_status = I2C_BUSY;
}


void i2c0_transmit(uint8_t slave_addr, uint8_t len, volatile bool_t* finished) {
  i2c0_trx = 0;
  i2c0_init_transmit(slave_addr, len, finished);
  i2c0_stop_after_transmit = TRUE; /** Default "historic" behaviour */
  I2c0SendStart();
}

void i2c0_transmit_no_stop(uint8_t slave_addr, uint8_t len, volatile bool_t* finished) {
  i2c0_trx = 0;
  i2c0_init_transmit(slave_addr, len, finished);
  i2c0_stop_after_transmit = FALSE; /** Default "historic" behaviour */
  I2c0SendStart();
}

void i2c0_transceive(uint8_t slave_addr, uint8_t len_w, uint16_t len_r, volatile bool_t* finished) {
  i2c0_trx = 1;
  i2c0_len_w = len_w;
  i2c0_len_r = len_r;
  i2c0_slave_addr = slave_addr & ~I2C_RECEIVE;
  i2c0_finished = finished;
  i2c0_status = I2C_BUSY;
  I2c0SendStart();
}


#endif /* USE_I2C0 */

#ifdef USE_I2C1

struct i2c_periph i2c1;

void i2c1_init(void) {
  i2c_init(&i2c2);
  i2c2_hw_init();
}

#endif /* USE_I2C1 */


#ifdef USE_I2C2

#include "booz/booz2_debug.h"

struct i2c_periph i2c2;

void i2c2_init(void) {
  i2c_init(&i2c2);
  i2c2_hw_init();
}

#endif /* USE_I2C2 */

void   i2c_init(struct i2c_periph* p) {
  p->trans_insert_idx = 0;
  p->trans_extract_idx = 0;
  p->status = I2CIdle;
}


