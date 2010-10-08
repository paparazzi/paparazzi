#include "peripherals/booz_hmc5843.h"

#include "i2c.h"

struct Hmc5843 hmc5843;
struct i2c_transaction hmc5843_i2c_trans;

void hmc5843_init(void) {
  hmc5843.status = HMC5843_UNINITIALIZED1;
  hmc5843.i2c_done = TRUE;

	hmc5843_i2c_trans.status = I2CTransSuccess;
	hmc5843_i2c_trans.slave_addr = HMC5843_ADDR;
	hmc5843_i2c_trans.stop_after_transmit = TRUE;
}

void hmc5843_periodic(void) {
  
  if (hmc5843_i2c_trans.status == I2CTransPending) return;

  switch (hmc5843.status) {
  case HMC5843_UNINITIALIZED1:
    hmc5843_i2c_trans.buf[0] = HMC5843_REG_CFGA;  // set to rate to 50Hz
    hmc5843_i2c_trans.buf[1] = 0x00 | (0x06 << 2);
    hmc5843_i2c_trans.type = I2CTransTx;
    hmc5843_i2c_trans.len_w = 2;
    i2c_submit(&i2c2, &hmc5843_i2c_trans);
    hmc5843.status = HMC5843_UNINITIALIZED2;
    break;
  case HMC5843_UNINITIALIZED2:
    hmc5843_i2c_trans.buf[0] = HMC5843_REG_CFGB;  // set to gain to 1 Gauss
    hmc5843_i2c_trans.buf[1] = 0x01<<5;
    hmc5843_i2c_trans.type = I2CTransTx;
    hmc5843_i2c_trans.len_w = 2;
    i2c_submit(&i2c2, &hmc5843_i2c_trans);
    hmc5843.status = HMC5843_UNINITIALIZED3;
    break;
  case HMC5843_UNINITIALIZED3:
    hmc5843_i2c_trans.buf[0] = HMC5843_REG_MODE;  // set to continuous mode
    hmc5843_i2c_trans.buf[1] = 0x00;
    hmc5843_i2c_trans.type = I2CTransTx;
    hmc5843_i2c_trans.len_w = 2;
    i2c_submit(&i2c2, &hmc5843_i2c_trans);
    hmc5843.status = HMC5843_IDLE;
    break;
  case HMC5843_IDLE:
    hmc5843_i2c_trans.type = I2CTransRx;
    hmc5843_i2c_trans.len_w = 6;
    i2c_submit(&i2c2, &hmc5843_i2c_trans);
    hmc5843.status = HMC5843_READING;
    break;
  default:
    /* FIXME : report error */
    break;
  }

}

