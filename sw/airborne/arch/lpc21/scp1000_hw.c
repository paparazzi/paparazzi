#include "scp1000.h"

#ifndef SCP1000_NO_EINT
static void EXTINT_ISR(void) __attribute__((naked));
#endif

void scp1000_hw_init(void) {
 /* configure SS pin */
  SetBit(SCP_SS_IODIR, SCP_SS_PIN); /* pin is output  */
  Scp1000Unselect();                /* pin idles high */
 
#ifndef SCP1000_NO_EINT
  /* configure DRDY pin */
  /* connected pin to EXINT */ 
  SCP_DRDY_PINSEL |= SCP_DRDY_PINSEL_VAL << SCP_DRDY_PINSEL_BIT;
  SetBit(EXTMODE, SCP_DRDY_EINT); /* EINT is edge trigered */
  SetBit(EXTPOLAR,SCP_DRDY_EINT); /* EINT is trigered on rising edge */
  SetBit(EXTINT,SCP_DRDY_EINT);   /* clear pending EINT */

  /* initialize interrupt vector */
  VICIntSelect &= ~VIC_BIT( SCP_DRDY_VIC_IT );  /* select EINT as IRQ source */
  VICIntEnable = VIC_BIT( SCP_DRDY_VIC_IT );    /* enable it */
  VICVectCntl10 = VIC_ENABLE | SCP_DRDY_VIC_IT;
  VICVectAddr10 = (uint32_t)EXTINT_ISR;         /* address of the ISR */
#endif
}

#ifndef SCP1000_NO_EINT
void EXTINT_ISR(void) {
  ISR_ENTRY();

  ASSERT((scp1000_status == SCP1000_STA_WAIT_EOC),
	 DEBUG_SCP1000, SCP1000_ERR_STATUS);

  scp1000_status = SCP1000_STA_GOT_EOC;

  SetBit(EXTINT,SCP_DRDY_EINT); /* clear EINT2 */
  VICVectAddr = 0x00000000;     /* clear this interrupt from the VIC */
  ISR_EXIT();
}
#endif
