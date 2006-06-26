#include "max1167.h"

volatile uint8_t max1167_data_available;
uint16_t max1167_values[MAX1167_NB_CHAN];

static void EXTINT0_ISR(void) __attribute__((naked));

extern void max1167_init( void ) {
  /* CS pin is output */
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

  max1167_data_available = FALSE;

}

void max1167_read( void ) {
  /* select max1167 */ 
  Max1167Select();
  /* enable SPI */
  SpiEnable();
  /* write control byte - wait for eoc on extint0 */
  uint8_t control_byte = 1 << 0 | 1 << 3 | 2 << 5;
  SpiSend(control_byte);
}

void EXTINT0_ISR(void) {
  ISR_ENTRY();
  
  /* read dummy control byte reply */
  uint8_t foo __attribute__ ((unused)) = SSPDR;
  /* trigger 6 bytes read */
  SSPDR = 0;
  SSPDR = 0;
  SSPDR = 0;
  SSPDR = 0;
  SSPDR = 0;
  SSPDR = 0;
  /* enable timeout interrupt */
  SpiEnableRti();
  /* clear extint0 */
  SetBit(EXTINT, MAX1167_EOC_EINT);

  VICVectAddr = 0x00000000; /* clear this interrupt from the VIC */
  ISR_EXIT();
}
