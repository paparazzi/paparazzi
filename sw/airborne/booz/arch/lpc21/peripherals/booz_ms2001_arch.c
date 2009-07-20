/* PNI ms2001 connected on SPI1 */
/*
  IMU b2
  SS on P1.28
  RESET on P1.19
  DRDY on P0.30 ( EINT3)
*/

#include "peripherals/booz_ms2001.h"

volatile uint8_t ms2001_cur_axe;

static void EXTINT_ISR(void) __attribute__((naked));

void ms2001_arch_init( void ) {

  /* configure SS pin */
  Ms2001Unselect();                       /* pin idles high */
  SetBit(MS2001_SS_IODIR, MS2001_SS_PIN); /* pin is output  */

  /* configure RESET pin */
  Ms2001Reset();                                /* pin idles low  */
  SetBit(MS2001_RESET_IODIR, MS2001_RESET_PIN); /* pin is output  */

  /* configure DRDY pin */
  /* connected pin to EXINT */ 
  MS2001_DRDY_PINSEL |= MS2001_DRDY_PINSEL_VAL << MS2001_DRDY_PINSEL_BIT;
  SetBit(EXTMODE, MS2001_DRDY_EINT); /* EINT is edge trigered */
  SetBit(EXTPOLAR,MS2001_DRDY_EINT); /* EINT is trigered on rising edge */
  SetBit(EXTINT,MS2001_DRDY_EINT);   /* clear pending EINT */
  
  /* initialize interrupt vector */
  VICIntSelect &= ~VIC_BIT( MS2001_DRDY_VIC_IT );                       /* select EINT as IRQ source */
  VICIntEnable = VIC_BIT( MS2001_DRDY_VIC_IT );                         /* enable it                 */
  _VIC_CNTL(MS2001_DRDY_VIC_SLOT) = VIC_ENABLE | MS2001_DRDY_VIC_IT;
  _VIC_ADDR(MS2001_DRDY_VIC_SLOT) = (uint32_t)EXTINT_ISR;         // address of the ISR 

}

void EXTINT_ISR(void) {
  ISR_ENTRY();
  /* no, we won't do anything asynchronously, so just notify */
  ms2001_status = MS2001_GOT_EOC;
  /* clear EINT */
  EXTINT = (1<<MS2001_DRDY_EINT);
  VICVectAddr = 0x00000000;    /* clear this interrupt from the VIC */
  ISR_EXIT();
}

