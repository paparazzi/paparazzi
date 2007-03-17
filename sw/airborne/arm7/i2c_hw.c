#include "i2c.h"


#include "led.h"

#include "std.h"

#include "interrupt_hw.h"



void i2c0_ISR(void) __attribute__((naked));


/* SDA0 on P0.3 */
/* SCL0 on P0.2 */
void i2c_hw_init ( void ) {

  /* set P0.2 and P0.3 to I2C0 */
  PINSEL0 |= 1 << 4 | 1 << 6;
  /* clear all flags */
  I2C0CONCLR = _BV(AAC) | _BV(SIC) | _BV(STAC) | _BV(I2ENC);
  /* enable I2C */
  I2C0CONSET = _BV(I2EN);
  /* set bitrate */
  I2C0SCLL = 200;  
  I2C0SCLH = 200;  
  
  // initialize the interrupt vector
  VICIntSelect &= ~VIC_BIT(VIC_I2C0);  // I2C0 selected as IRQ
  VICIntEnable = VIC_BIT(VIC_I2C0);    // I2C0 interrupt enabled
  VICVectCntl9 = VIC_ENABLE | VIC_I2C0;
  VICVectAddr9 = (uint32_t)i2c0_ISR;    // address of the ISR
}

#define I2C_DATA_REG I2C0DAT
#define I2C_STATUS_REG I2C0STAT

void i2c0_ISR(void)
{
  ISR_ENTRY();

  uint32_t state = I2C_STATUS_REG;
  I2cAutomaton(state);
  I2cClearIT();
  
  VICVectAddr = 0x00000000;             // clear this interrupt from the VIC
  ISR_EXIT();                           // recover registers and return
}
