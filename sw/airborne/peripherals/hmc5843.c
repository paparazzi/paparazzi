#include "peripherals/hmc5843.h"

#include "led.h"

#define HMC5843_TIMEOUT 10

#define bswap_16(x)   ((((x) & 0xFF00) >> 8) | (((x) & 0x00FF) << 8))

struct Hmc5843 hmc5843;
void exti9_5_irq_handler(void);

void hmc5843_init(void)
{
  hmc5843.i2c_trans.status = I2CTransSuccess;
  hmc5843.i2c_trans.slave_addr = HMC5843_ADDR;

  hmc5843_arch_init();
}

static void hmc_send_config(uint8_t _init)
{
  switch (_init)
  {
  case 0:
    hmc5843.i2c_trans.type = I2CTransTx;
    hmc5843.i2c_trans.buf[0] = HMC5843_REG_CFGA;  // set to rate to 50Hz
    hmc5843.i2c_trans.buf[1] = 0x00 | (0x06 << 2);
    hmc5843.i2c_trans.len_w = 2;
    i2c_submit(&i2c2,&hmc5843.i2c_trans);
    break;
  case 1:
    hmc5843.i2c_trans.type = I2CTransTx;
    hmc5843.i2c_trans.buf[0] = HMC5843_REG_CFGB;  // set to gain to 1 Gauss
    hmc5843.i2c_trans.buf[1] = 0x01<<5;
    hmc5843.i2c_trans.len_w = 2;
    i2c_submit(&i2c2,&hmc5843.i2c_trans);
  break;
  case 2:
    hmc5843.i2c_trans.type = I2CTransTx;
    hmc5843.i2c_trans.buf[0] = HMC5843_REG_MODE;  // set to continuous mode
    hmc5843.i2c_trans.buf[1] = 0x00;
    hmc5843.i2c_trans.len_w = 2;
    i2c_submit(&i2c2,&hmc5843.i2c_trans);
  break;
  default: 
    hmc5843.i2c_trans.type = I2CTransTxRx;
    hmc5843.i2c_trans.len_r = 6;
    hmc5843.i2c_trans.len_w = 1;
    hmc5843.i2c_trans.buf[0] = HMC5843_REG_DATXM;
    i2c_submit(&i2c2, &hmc5843.i2c_trans);
  }
}

void hmc5843_idle_task(void)
{
  if (hmc5843.timeout > HMC5843_TIMEOUT)
  {
    hmc5843.timeout = 0;
    LED_TOGGLE(4);
  }

  // Wait for I2C transaction object to be released by the I2C driver before changing anything
  if ((hmc5843.i2c_trans.status == I2CTransFailed) || (hmc5843.i2c_trans.status == I2CTransSuccess))
  {
    if (hmc5843.initialized < 4)
    {
       if (hmc5843.i2c_trans.status == I2CTransSuccess)
       {
         hmc5843.initialized++;
       }
       hmc_send_config( hmc5843.initialized );
     }
     else
     {

 	 
/*
  // If transaction succeeded
  if (hmc5843.i2c_trans.status == I2CTransSuccess) 
  {
    memcpy(hmc5843.data.buf, (const void *) hmc5843.i2c_trans.buf, 6);
    for (int i = 0; i < 3; i++) {
      hmc5843.data.value[i] = bswap_16(hmc5843.data.value[i]);
    }
    hmc5843.data_available = TRUE;
  }

  // Start a new one
  if (hmc5843.timeout > HMC5843_TIMEOUT)
  {

    hmc5843.timeout = 0;
*/
      hmc5843.i2c_trans.type = I2CTransTxRx;
      hmc5843.i2c_trans.len_r = 6;
      hmc5843.i2c_trans.len_w = 1;
      hmc5843.i2c_trans.buf[0] = HMC5843_REG_DATXM;
      i2c_submit(&i2c2, &hmc5843.i2c_trans);
    }
  }

}

void hmc5843_periodic(void)
{
  hmc5843.timeout++;
}
