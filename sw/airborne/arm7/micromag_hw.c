/* PNI micromag3 connected on SPI1 */
/* 
   Twisted Logic
   SS    on P0.20
   RESET on P0.29 
   DRDY  on P0.30 ( EINT3 )
*/

/* 
   IMU v3
   SS on P0.20
   RESET on P1.21
   DRDY on P0.15 ( EINT2 )
*/

/*
  IMU b2
  SS on P1.28
  RESET on P1.19
  DRDY on P0.30 ( EINT3)
*/

#include "micromag.h"

volatile uint8_t micromag_cur_axe;

static void EXTINT_ISR(void) __attribute__((naked));

void micromag_hw_init( void ) {

  /* configure SS pin */
  SetBit(MM_SS_IODIR, MM_SS_PIN); /* pin is output  */
  MmUnselect();                   /* pin idles high */

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

#include "uart.h"
#include "messages.h"
#include "downlink.h"
#include "led.h"

void EXTINT_ISR(void) {
  ISR_ENTRY();
  LED_ON(2);
  micromag_status = MM_GOT_EOC;
  /* clear EINT */
  SetBit(EXTINT,MM_DRDY_EINT);

  VICVectAddr = 0x00000000;    /* clear this interrupt from the VIC */
  ISR_EXIT();
}

