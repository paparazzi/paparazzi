/* PNI micromag3 connected on SPI1 */
/* 
   Twisted Logic
   SS    on P0.20
   RESET on P0.29 
   DRDY  on P0.30 ( EINT3 )
*/

/* 
   IMU
   SS on P0.20
   RESET on P1.21
   DRDY on P0.15 ( EINT2 )
*/

#include "micromag.h"


volatile uint8_t micromag_cur_axe;

static void EXTINT2_ISR(void) __attribute__((naked));

void micromag_hw_init( void ) {
  /* configure SS pin */
  SetBit(MM_SS_IODIR, MM_SS_PIN); /* pin is output  */
  MmUnselect();                   /* pin idles high */

  /* configure RESET pin */
  SetBit(MM_RESET_IODIR, MM_RESET_PIN); /* pin is output  */
  MmReset();                            /* pin idles low  */

  /* configure DRDY pin */
  /* connected pin to EINT2 */ 
  MM_DRDY_PINSEL |= MM_DRDY_PINSEL_VAL << MM_DRDY_PINSEL_BIT;
  SetBit(EXTMODE, MM_DRDY_EINT); /* EINT is edge trigered */
  SetBit(EXTPOLAR,MM_DRDY_EINT); /* EINT is trigered on rising edge */
  SetBit(EXTINT,MM_DRDY_EINT);   /* clear pending EINT */
  
  /* initialize interrupt vector */
  VICIntSelect &= ~VIC_BIT( VIC_EINT3 );  /* select EINT2 as IRQ source */
  VICIntEnable = VIC_BIT( VIC_EINT3 );    /* enable it */
  VICVectCntl9 = VIC_ENABLE | VIC_EINT3;
  VICVectAddr9 = (uint32_t)EXTINT2_ISR;    // address of the ISR 
}

void micromag_read( void ) {
  if (micromag_status == MM_IDLE ) {
    MmSelect();
    SpiEnable();
    SpiDisableRti();
    micromag_cur_axe = 0;
    micromag_status = MM_BUSY;
    MmTriggerRead();
  }
}

void EXTINT2_ISR(void) {
  ISR_ENTRY();
  /* read dummy control byte reply */
  uint8_t foo __attribute__ ((unused)) = SSPDR;
  /* trigger 2 bytes read */
  SSPDR = 0;
  SSPDR = 0;
  /* enable timeout interrupt */
  SpiEnableRti();
  /* clear EINT2 */
  SetBit(EXTINT,MM_DRDY_EINT); /* clear EINT2 */

  VICVectAddr = 0x00000000;    /* clear this interrupt from the VIC */
  ISR_EXIT();
}

