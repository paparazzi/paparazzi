#include "peripherals/hmc5843.h"

#include "led.h"

#define HMC5843_TIMEOUT 100

#define bswap_16(x)   ((((x) & 0xFF00) >> 8) | (((x) & 0x00FF) << 8))

struct Hmc5843 hmc5843;
void exti9_5_irq_handler(void);

struct i2c_transaction i2c_test2;


void hmc5843_init(void)
{
  hmc5843.i2c_trans.status = I2CTransSuccess;
  hmc5843.i2c_trans.slave_addr = HMC5843_ADDR;

  hmc5843_arch_init();
  hmc5843.initialized = 0;
  hmc5843.timeout = 10000;
}

static void hmc_send_config(uint8_t _init)
{
  //i2c_test2.slave_addr = 0x90;
  i2c_test2.type = I2CTransTx;
  i2c_test2.slave_addr = 0x00;
  i2c_test2.len_w = 1;
  i2c_test2.buf[0] = 0x06;

  hmc5843.i2c_trans.slave_addr = HMC5843_ADDR;
  hmc5843.i2c_trans.len_w = 0;
  hmc5843.i2c_trans.len_r = 0;
  switch (_init)
  {
  case 1:
    hmc5843.i2c_trans.type = I2CTransTx;
    hmc5843.i2c_trans.buf[0] = HMC5843_REG_CFGA;  // set to rate to 50Hz
    hmc5843.i2c_trans.buf[1] = 0x00 | (0x06 << 2);
    hmc5843.i2c_trans.buf[2] = 0x01<<5;
    hmc5843.i2c_trans.buf[3] = 0x00;
    hmc5843.i2c_trans.len_w = 4;
    i2c_submit(&i2c2,&hmc5843.i2c_trans);
    break;
  case 2:
    hmc5843.i2c_trans.type = I2CTransTx;
    hmc5843.i2c_trans.buf[0] = HMC5843_REG_CFGB;  // set to gain to 1 Gauss
    hmc5843.i2c_trans.buf[1] = 0x01<<5;
    hmc5843.i2c_trans.len_w = 2;
    i2c_submit(&i2c2,&hmc5843.i2c_trans);
  break;
  case 3:
    hmc5843.i2c_trans.type = I2CTransTx;
    hmc5843.i2c_trans.buf[0] = HMC5843_REG_CFGA;  // set to continuous mode
    hmc5843.i2c_trans.len_w = 1;
    i2c_submit(&i2c2,&hmc5843.i2c_trans);
  break;
  case 4:
    hmc5843.i2c_trans.type = I2CTransRx;
    hmc5843.i2c_trans.len_r = 1;
    i2c_submit(&i2c2,&hmc5843.i2c_trans);
  break;
  case 5:
    hmc5843.i2c_trans.type = I2CTransRx;
    hmc5843.i2c_trans.len_r = 2;
    i2c_submit(&i2c2,&hmc5843.i2c_trans);
  break;
  case 6:
    hmc5843.i2c_trans.type = I2CTransRx;
    hmc5843.i2c_trans.len_r = 3;
    i2c_submit(&i2c2,&hmc5843.i2c_trans);
  break;
  case 7:
    hmc5843.i2c_trans.type = I2CTransRx;
    hmc5843.i2c_trans.len_r = 4;
    i2c_submit(&i2c2,&hmc5843.i2c_trans);
  break;
  case 8:
    hmc5843.i2c_trans.type = I2CTransRx;
    hmc5843.i2c_trans.len_r = 5;
    i2c_submit(&i2c2,&hmc5843.i2c_trans);
  break;
  case 9:
    // bad addr
    hmc5843.i2c_trans.slave_addr = HMC5843_ADDR + 2;
    hmc5843.i2c_trans.type = I2CTransTx;
    hmc5843.i2c_trans.len_w = 1;
    i2c_submit(&i2c2,&hmc5843.i2c_trans);
  break;
  case 10:
    // 2 consecutive
    hmc5843.i2c_trans.type = I2CTransTx;
    hmc5843.i2c_trans.buf[0] = HMC5843_REG_CFGA;  // set to continuous mode
    hmc5843.i2c_trans.len_w = 1;
    i2c_submit(&i2c2,&hmc5843.i2c_trans);
    //i2c_submit(&i2c2,&i2c_test2);
  break;
  case 11:
    hmc5843.i2c_trans.slave_addr = HMC5843_ADDR;
    hmc5843.i2c_trans.type = I2CTransTxRx;
    hmc5843.i2c_trans.len_r = 1;
    hmc5843.i2c_trans.len_w = 1;
    hmc5843.i2c_trans.buf[0] = HMC5843_REG_DATXM;
    i2c_submit(&i2c2, &hmc5843.i2c_trans);
  break;
  case 12:
    hmc5843.i2c_trans.slave_addr = HMC5843_ADDR;
    hmc5843.i2c_trans.type = I2CTransTxRx;
    hmc5843.i2c_trans.len_r = 2;
    hmc5843.i2c_trans.len_w = 1;
    hmc5843.i2c_trans.buf[0] = HMC5843_REG_DATXM;
    i2c_submit(&i2c2, &hmc5843.i2c_trans);
  break;
  case 13:
    hmc5843.i2c_trans.slave_addr = HMC5843_ADDR;
    hmc5843.i2c_trans.type = I2CTransTxRx;
    hmc5843.i2c_trans.len_r = 3;
    hmc5843.i2c_trans.len_w = 1;
    hmc5843.i2c_trans.buf[0] = HMC5843_REG_DATXM;
    i2c_submit(&i2c2, &hmc5843.i2c_trans);
  break;
  case 14:
    hmc5843.i2c_trans.slave_addr = HMC5843_ADDR;
    hmc5843.i2c_trans.type = I2CTransTxRx;
    hmc5843.i2c_trans.len_r = 4;
    hmc5843.i2c_trans.len_w = 1;
    hmc5843.i2c_trans.buf[0] = HMC5843_REG_DATXM;
    i2c_submit(&i2c2, &hmc5843.i2c_trans);
  break;
  case 15:
    hmc5843.i2c_trans.slave_addr = HMC5843_ADDR;
    hmc5843.i2c_trans.type = I2CTransTxRx;
    hmc5843.i2c_trans.len_r = 4;
    hmc5843.i2c_trans.len_w = 2;
    hmc5843.i2c_trans.buf[0] = HMC5843_REG_DATXM;
    i2c_submit(&i2c2, &hmc5843.i2c_trans);
  break;
  default: 
    hmc5843.i2c_trans.slave_addr = HMC5843_ADDR;
    hmc5843.i2c_trans.type = I2CTransTxRx;
    hmc5843.i2c_trans.len_r = 5;
    hmc5843.i2c_trans.len_w = 1;
    hmc5843.i2c_trans.buf[0] = HMC5843_REG_DATXM;
    i2c_submit(&i2c2, &hmc5843.i2c_trans);
  }
}

void hmc5843_idle_task(void)
{
  if (i2c_idle(&i2c2))
  {
    LED_ON(7);	// green = idle
    LED_OFF(6);
  }
  else
  {
    LED_ON(6); // red = busy
    LED_OFF(7);
  }

  // Wait for I2C transaction object to be released by the I2C driver before changing anything
  if (i2c_idle(&i2c2))
  {
	  if ((hmc5843.i2c_trans.status == I2CTransFailed) || (hmc5843.i2c_trans.status == I2CTransSuccess))
	  {
	    LED_ON(5);
	    LED_OFF(4);
	    if (hmc5843.initialized < 16)
	    {
	       hmc5843.initialized++;
	    }
	    else
	    {
	      hmc5843.initialized = 1;

	      i2c2_setbitrate(hmc5843.timeout);

	      hmc5843.timeout += 10000;
              if (hmc5843.timeout > 800000)
              {
                hmc5843.timeout = 10000;
              }
	    }

	    //if (hmc5843.initialized < 4)
	    {
	      hmc_send_config( hmc5843.initialized ); 	 
	    }

	  }
	  else
	  {
	    LED_ON(4);
	    LED_OFF(5);
	  }
  }
}

void hmc5843_periodic(void)
{
  //hmc5843.timeout++;
}
