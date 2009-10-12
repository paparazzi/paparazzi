/* PNI micromag3 connected on SPI1 */
/* 
   Tiny2 (fixed wing)
   SS    on P0.20 (SSEL)
   RESET on P0.29 (ADC5)
   DRDY  on P0.16 ( EINT0 )
*/

#include "led.h"
#include "micromag_fw.h"

volatile uint8_t micromag_cur_axe;

static void EXTINT_ISR(void) __attribute__((naked));

void micromag_hw_init( void ) {

  MmUnselect();                   /* pin idles high */
  /* configure SS pin */
  SetBit(MM_SS_IODIR, MM_SS_PIN); /* pin is output  */

  /* configure RESET pin */
  SetBit(MM_RESET_IODIR, MM_RESET_PIN); /* pin is output  */
  MmReset();                            /* pin idles low  */

  /* configure DRDY pin */
  /* connected pin to EXINT */ 
  MM_DRDY_PINSEL |= MM_DRDY_PINSEL_VAL << MM_DRDY_PINSEL_BIT;
  SetBit(EXTMODE, MM_DRDY_EINT); /* EINT is edge trigered */
  SetBit(EXTPOLAR,MM_DRDY_EINT); /* EINT is trigered on rising edge */
  SetBit(EXTINT,MM_DRDY_EINT);   /* clear pending EINT */
  
  /* initialize interrupt vector */
  VICIntSelect &= ~VIC_BIT( MM_DRDY_VIC_IT );                       /* select EINT as IRQ source */
  VICIntEnable = VIC_BIT( MM_DRDY_VIC_IT );                         /* enable it                 */
  _VIC_CNTL(MICROMAG_DRDY_VIC_SLOT) = VIC_ENABLE | MM_DRDY_VIC_IT;
  _VIC_ADDR(MICROMAG_DRDY_VIC_SLOT) = (uint32_t)EXTINT_ISR;         // address of the ISR 
}

void EXTINT_ISR(void) {
  ISR_ENTRY();
//LED_TOGGLE(3); 

  /* no, we won't do anything asynchronously, so just notify */
  micromag_status = MM_GOT_EOC;
  /* clear EINT */
  SetBit(EXTINT,MM_DRDY_EINT);
//  EXTINT = (1<<MM_DRDY_EINT);
  VICVectAddr = 0x00000000;    /* clear this interrupt from the VIC */
  ISR_EXIT();
}

