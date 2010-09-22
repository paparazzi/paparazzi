
/* MAX11040 connected on SPI1

   SS    on P0.20 (SSEL)
   DRDY  on P0.16 (EINT0)
*/


#include "led.h"
#include "max11040.h"

#ifdef LOGGER
extern unsigned int getclock(void);
#endif

volatile uint8_t num_irqs = 0;

static void EXTINT_ISR(void) __attribute__((naked));

void max11040_hw_init( void ) {

  /* configure DRDY pin */
  /* connected pin to EXINT */ 
  MAXM_DRDY_PINSEL |= MAXM_DRDY_PINSEL_VAL << MAXM_DRDY_PINSEL_BIT;
  SetBit(EXTMODE, MAXM_DRDY_EINT);     /* EINT is edge trigered */
  ClearBit(EXTPOLAR, MAXM_DRDY_EINT);  /* EINT is trigered on falling edge */
  SetBit(EXTINT, MAXM_DRDY_EINT);      /* clear pending EINT */
  
  /* initialize interrupt vector */
  VICIntSelect &= ~VIC_BIT( MAXM_DRDY_VIC_IT );                       /* select EINT as IRQ source */
  VICIntEnable = VIC_BIT( MAXM_DRDY_VIC_IT );                         /* enable it                 */
  _VIC_CNTL(MAX11040_DRDY_VIC_SLOT) = VIC_ENABLE | MAXM_DRDY_VIC_IT;
  _VIC_ADDR(MAX11040_DRDY_VIC_SLOT) = (uint32_t)EXTINT_ISR;           /* address of the ISR        */
}

void EXTINT_ISR(void) {
  ISR_ENTRY();

  if (num_irqs++ == 5) 
  {
    /* switch SSEL P0.20 to be used as GPIO */
    PINSEL1 &= ~(3 << 8);
    IO0DIR |= 1 << 20;
    max11040_status = MAX11040_DATA2;
  }

  if (max11040_status == MAX11040_DATA2) {

//LED_TOGGLE(2);
//LED_TOGGLE(3);

#ifdef LOGGER
    max11040_timestamp[max11040_buf_in] = getclock();
#endif

    MaxmSelect();

    /* read data */
    SSP_Send(0xF0);
    SSP_Send(0x00);
    SSP_Send(0x00);
    SSP_Send(0x00);
    SSP_Send(0x00);
    SSP_Send(0x00);
    SSP_Send(0x00);

    max11040_count = 0;
  }

  /* clear EINT */
  SetBit(EXTINT, MAXM_DRDY_EINT);

  VICVectAddr = 0x00000000;    /* clear this interrupt from the VIC */
  ISR_EXIT();
}

