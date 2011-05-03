#include "peripherals/hmc5843.h"

#define HMC5843_TIMEOUT 100

#define bswap_16(x)   ((((x) & 0xFF00) >> 8) | (((x) & 0x00FF) << 8))

struct Hmc5843 hmc5843;
void exti9_5_irq_handler(void);

void hmc5843_init(void)
{
  hmc5843.i2c_trans.status = I2CTransSuccess;
  hmc5843.i2c_trans.slave_addr = HMC5843_ADDR;

  hmc5843_arch_init();
}

// blocking, only intended to be called for initialization
static void send_config(void)
{
  hmc5843.i2c_trans.type = I2CTransTx;
  hmc5843.i2c_trans.buf[0] = HMC5843_REG_CFGA;  // set to rate to 50Hz
  hmc5843.i2c_trans.buf[1] = 0x00 | (0x06 << 2);
  hmc5843.i2c_trans.len_w = 2;
  i2c_submit(&i2c2,&hmc5843.i2c_trans);
  while(hmc5843.i2c_trans.status == I2CTransPending);

  hmc5843.i2c_trans.type = I2CTransTx;
  hmc5843.i2c_trans.buf[0] = HMC5843_REG_CFGB;  // set to gain to 1 Gauss
  hmc5843.i2c_trans.buf[1] = 0x01<<5;
  hmc5843.i2c_trans.len_w = 2;
  i2c_submit(&i2c2,&hmc5843.i2c_trans);
  while(hmc5843.i2c_trans.status == I2CTransPending);

  hmc5843.i2c_trans.type = I2CTransTx;
  hmc5843.i2c_trans.buf[0] = HMC5843_REG_MODE;  // set to continuous mode
  hmc5843.i2c_trans.buf[1] = 0x00;
  hmc5843.i2c_trans.len_w = 2;
  i2c_submit(&i2c2,&hmc5843.i2c_trans);
  while(hmc5843.i2c_trans.status == I2CTransPending);

}

void hmc5843_idle_task(void)
{
  if (hmc5843.i2c_trans.status == I2CTransFailed) {
    hmc5843.sent_tx = 0;
    hmc5843.sent_rx = 0;
  }

  if (hmc5843.i2c_trans.status == I2CTransRunning || hmc5843.i2c_trans.status == I2CTransPending) return;

  if (hmc5843.initialized && mag_eoc() && !hmc5843.sent_tx && !hmc5843.sent_rx) {
    if (i2c2.status == I2CIdle && i2c_idle(&i2c2)) {
      hmc5843.i2c_trans.type = I2CTransTx;
      hmc5843.i2c_trans.len_w = 1;
      hmc5843.i2c_trans.buf[0] = 0x3;
      i2c_submit(&i2c2, &hmc5843.i2c_trans);
      hmc5843.sent_tx = 1;
      return;
    }
  }

  if (hmc5843.sent_tx) {
    hmc5843.i2c_trans.type = I2CTransRx;
    hmc5843.i2c_trans.len_r = 6;
    hmc5843.i2c_trans.len_w = 1;
    hmc5843.i2c_trans.buf[0] = 0x3;
    i2c_submit(&i2c2, &hmc5843.i2c_trans);
    hmc5843.sent_rx = 1;
    hmc5843.sent_tx = 0;
    return;
  }

  if (hmc5843.sent_rx && hmc5843.i2c_trans.status == I2CTransSuccess) {
    hmc5843.sent_rx = 0;
    hmc5843.sent_tx = 0;
    hmc5843.timeout = 0;
    hmc5843.data_available = TRUE;
    memcpy(hmc5843.data.buf, (const void *) hmc5843.i2c_trans.buf, 6);
    for (int i = 0; i < 3; i++) {
      hmc5843.data.value[i] = bswap_16(hmc5843.data.value[i]);
    }
  }
}

void hmc5843_periodic(void)
{
  if (!hmc5843.initialized) {
    send_config();
    hmc5843.initialized = TRUE;
  } else if (hmc5843.timeout++ > HMC5843_TIMEOUT && i2c2.status == I2CIdle && i2c_idle(&i2c2)){
#ifdef USE_HMC59843_ARCH_RESET
    hmc5843_arch_reset();
#endif
    hmc5843.i2c_trans.type = I2CTransTx;
    hmc5843.i2c_trans.len_w = 1;
    hmc5843.i2c_trans.buf[0] = 0x3;
    i2c_submit(&i2c2, &hmc5843.i2c_trans);
    while(hmc5843.i2c_trans.status == I2CTransPending || hmc5843.i2c_trans.status == I2CTransRunning);

    hmc5843.i2c_trans.type = I2CTransRx;
    hmc5843.i2c_trans.len_r = 6;
    i2c_submit(&i2c2, &hmc5843.i2c_trans);
    while(hmc5843.i2c_trans.status == I2CTransPending || hmc5843.i2c_trans.status == I2CTransRunning);
    hmc5843.timeout = 0;
  }
}
