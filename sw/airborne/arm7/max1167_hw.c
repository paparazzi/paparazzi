#include "max1167.h"

#include "led.h"

uint8_t max1167_error;

static void EXTINT0_ISR(void) __attribute__((naked));

extern void max1167_hw_init( void ) {
  
  max1167_error = MAX1167_ERR_NONE;

  /* SS pin is output */
  SetBit(MAX1167_SS_IODIR, MAX1167_SS_PIN);
  /* unselected max1167 */
  Max1167Unselect();

  /* connect P0.16 to extint0 (EOC) */
  MAX1167_EOC_PINSEL |= MAX1167_EOC_PINSEL_VAL << MAX1167_EOC_PINSEL_BIT;
  /* extint0 is edge trigered */
  SetBit(EXTMODE, MAX1167_EOC_EINT);
  /* extint0 is trigered on falling edge */
  ClearBit(EXTPOLAR, MAX1167_EOC_EINT);
  /* clear pending extint0 before enabling interrupts */
  SetBit(EXTINT, MAX1167_EOC_EINT);

   /* initialize interrupt vector */
  VICIntSelect &= ~VIC_BIT( VIC_EINT0 );  // EXTINT0 selected as IRQ
  VICIntEnable = VIC_BIT( VIC_EINT0 );    // EXTINT0 interrupt enabled
  VICVectCntl8 = VIC_ENABLE | VIC_EINT0;
  VICVectAddr8 = (uint32_t)EXTINT0_ISR;    // address of the ISR 
}

#include "uart.h"
#include "messages.h"
#include "downlink.h"

void max1167_read( void ) {
  if (max1167_status == STA_MAX1167_IDLE) {
    /* select max1167 */ 
    Max1167Select();
    /* enable SPI */
    SpiEnable();
    /* write control byte - wait for RTI in spi handler */
    const uint8_t control_byte = 1 << 0 | 1 << 3 | 2 << 5;
    SpiSend(control_byte);
    /* enable timeout interrupt */
    //    SpiClearRti();	
    //    SpiEnableRti();
    max1167_status = STA_MAX1167_SENDING_REQ;
  }
  else {
    /* report overrun error */
    max1167_error = MAX1167_ERR_READ_OVERUN;
    DOWNLINK_SEND_DEBUG_MCU_LINK(&max1167_status,&max1167_error, &max1167_status);  
  }
}

void EXTINT0_ISR(void) {
  ISR_ENTRY();

  //  if (max1167_status == STA_MAX1167_WAIT_EOC) {
  if (max1167_status == STA_MAX1167_SENDING_REQ) {
    SpiEnable();
    /* trigger 6 bytes read */
    SpiSend(0);
    SpiSend(0);
    SpiSend(0);
    SpiSend(0);
    SpiSend(0);
    SpiSend(0);
    SpiClearRti();
    SpiEnableRti();
    /* here we could disable extint0 */
    /* and it would be re enabled when we enter WAIT_EOC */
    max1167_status = STA_MAX1167_READING_RES;
  }
  else {
    /* report error */
    max1167_error = MAX1167_ERR_SPURIOUS_EOC;
    DOWNLINK_SEND_DEBUG_MCU_LINK(&max1167_status,&max1167_error, &max1167_status);  
    /* reset */
    max1167_status = STA_MAX1167_IDLE;
    SpiDisable();		
    Max1167Unselect();
  }
  
  SetBit(EXTINT, MAX1167_EOC_EINT);   /* clear extint0 */
  VICVectAddr = 0x00000000; /* clear this interrupt from the VIC */

  ISR_EXIT();
}
