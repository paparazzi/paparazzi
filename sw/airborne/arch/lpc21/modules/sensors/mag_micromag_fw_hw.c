/* PNI micromag3 connected on SPI1 */
/*
   Tiny2 (fixed wing)
   SS    on P0.20 (SSEL)
   RESET on P0.29 (ADC5)
   DRDY  on P0.16 ( EINT0 )
*/

#include "led.h"
#include "modules/sensors/mag_micromag_fw_hw.h"
#include "modules/sensors/mag_micromag_fw.h"

volatile uint8_t micromag_cur_axe;

static void SSP_ISR(void) __attribute__((naked));
static void EXTINT_ISR(void) __attribute__((naked));

#warning "This driver should be updated to use the new SPI peripheral"

#ifndef SPI1_VIC_SLOT
#define SPI1_VIC_SLOT 7
#endif

static void SSP_ISR(void)
{
  ISR_ENTRY();

  MmOnSpiIt();

  VICVectAddr = 0x00000000; /* clear this interrupt from the VIC */
  ISR_EXIT();
}

void EXTINT_ISR(void)
{
  ISR_ENTRY();
//LED_TOGGLE(3);

  /* no, we won't do anything asynchronously, so just notify */
  micromag_status = MM_GOT_EOC;
  /* clear EINT */
  SetBit(EXTINT, MM_DRDY_EINT);
//  EXTINT = (1<<MM_DRDY_EINT);
  VICVectAddr = 0x00000000;    /* clear this interrupt from the VIC */
  ISR_EXIT();
}

void micromag_hw_init(void)
{
  /* setup pins for SSP (SCK, MISO, MOSI, SSEL) */
  PINSEL1 |= SSP_PINSEL1_SCK  | SSP_PINSEL1_MISO | SSP_PINSEL1_MOSI;

  /* setup SSP */
  SSPCR0 = SSPCR0_VAL;;
  SSPCR1 = SSPCR1_VAL;
  SSPCPSR = 0x02;

  /* initialize interrupt vector */
  VICIntSelect &= ~VIC_BIT(VIC_SPI1);    /* SPI1 selected as IRQ */
  VICIntEnable = VIC_BIT(VIC_SPI1);      /* enable it            */
  _VIC_CNTL(SPI1_VIC_SLOT) = VIC_ENABLE | VIC_SPI1;
  _VIC_ADDR(SPI1_VIC_SLOT) = (uint32_t)SSP_ISR;      /* address of the ISR   */

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
  SetBit(EXTPOLAR, MM_DRDY_EINT); /* EINT is trigered on rising edge */
  SetBit(EXTINT, MM_DRDY_EINT);  /* clear pending EINT */

  /* initialize interrupt vector */
  VICIntSelect &= ~VIC_BIT(MM_DRDY_VIC_IT);                         /* select EINT as IRQ source */
  VICIntEnable = VIC_BIT(MM_DRDY_VIC_IT);                           /* enable it                 */
  _VIC_CNTL(MICROMAG_DRDY_VIC_SLOT) = VIC_ENABLE | MM_DRDY_VIC_IT;
  _VIC_ADDR(MICROMAG_DRDY_VIC_SLOT) = (uint32_t)EXTINT_ISR;         // address of the ISR
}


