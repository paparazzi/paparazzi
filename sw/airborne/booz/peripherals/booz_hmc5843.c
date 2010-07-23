#include "peripherals/booz_hmc5843.h"

#include "i2c.h"

struct Hmc5843 hmc5843;

void hmc5843_init(void) {
  hmc5843.status = HMC5843_UNINITIALIZED1;
  hmc5843.i2c_done = TRUE;
}

void hmc5843_periodic(void) {
  
  if (!hmc5843.i2c_done) return;
  switch (hmc5843.status) {
  case HMC5843_UNINITIALIZED1:
    i2c2.buf[0] = HMC5843_REG_CFGA;  // set to rate to 50Hz
    i2c2.buf[1] = 0x00 | (0x06 << 2);
    i2c2_transmit(HMC5843_ADDR, 2, &hmc5843.i2c_done);
    hmc5843.status = HMC5843_UNINITIALIZED2;
    break;
  case HMC5843_UNINITIALIZED2:
    i2c2.buf[0] = HMC5843_REG_CFGB;  // set to gain to 1 Gauss
    i2c2.buf[1] = 0x01<<5;
    i2c2_transmit(HMC5843_ADDR, 2, &hmc5843.i2c_done);
    hmc5843.status = HMC5843_UNINITIALIZED3;
    break;
  case HMC5843_UNINITIALIZED3:
    i2c2.buf[0] = HMC5843_REG_MODE;  // set to continuous mode
    i2c2.buf[1] = 0x00;
    i2c2_transmit(HMC5843_ADDR, 2, &hmc5843.i2c_done);
    hmc5843.status = HMC5843_IDLE;
    break;
  case HMC5843_IDLE:
    i2c2_receive(HMC5843_ADDR, 6, &hmc5843.i2c_done);
    hmc5843.status = HMC5843_READING;
    break;
  default:
    /* FIXME : report error */
    break;
  }

}

