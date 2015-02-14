/*
 * Copyright (C) 2008-2009 Antoine Drouin <poinix@gmail.com>
 *
 * This file is part of paparazzi.
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 */

#include <inttypes.h>

#include "std.h"
#include "mcu.h"
#include "mcu_periph/sys_time.h"
#include "armVIC.h"
#include "LPC21xx.h"

static inline void main_init(void);
static inline void main_periodic_task(void);
static inline void main_event_task(void);


static inline void main_spi_init(void);
static void SPI0_ISR(void) __attribute__((naked));


int main(void)
{
  main_init();
  while (1) {
    if (sys_time_check_and_ack_timer(0)) {
      main_periodic_task();
    }
    main_event_task();
  }
  return 0;
}

static inline void main_init(void)
{
  mcu_init();
  sys_time_register_timer((1. / PERIODIC_FREQUENCY), NULL);
  main_spi_init();
  mcu_int_enable();
}

static inline void main_periodic_task(void)
{

}

static inline void main_event_task(void)
{

}

#define PINSEL0_SCK  (1<<8)
#define PINSEL0_MISO (1<<10)
#define PINSEL0_MOSI (1<<12)
#define PINSEL0_SSEL (1<<14)

#define S0SPCR_bit_enable  (0<<2)  /* 8 bits               */
#define S0SPCR_CPHA        (0<<3)  /* sample on first edge */
#define S0SPCR_CPOL        (0<<4)  /* clock idles low      */
#define S0SPCR_MSTR        (0<<5)  /* slave mode           */
#define S0SPCR_LSBF        (0<<6)  /* lsb first            */
#define S0SPCR_SPIE        (1<<7)  /* interrupt enable     */

#define S0SPCR_LSF_VAL (S0SPCR_bit_enable | S0SPCR_CPHA | \
                        S0SPCR_CPOL | S0SPCR_MSTR | \
                        S0SPCR_LSBF | S0SPCR_SPIE);

#define CPSDVSR 64

/* S0SPR bits */
#define ROVR 5
#define WCOL 6
#define SPIF 7
/* S0SPCR bits */
#define SPIE 7


static inline void main_spi_init(void)
{
  /* setup pins for SPI0 (SCK, MISO, MOSI, SS) */
  PINSEL0 |= PINSEL0_SCK | PINSEL0_MISO | PINSEL0_MOSI | PINSEL0_SSEL;

  S0SPCR = S0SPCR_LSF_VAL;
  S0SPCCR = CPSDVSR;

  /* initialize interrupt vector */
  VICIntSelect &= ~VIC_BIT(VIC_SPI0);   // SPI1 selected as IRQ
  VICIntEnable = VIC_BIT(VIC_SPI0);     // SPI1 interrupt enabled
  VICVectCntl7 = VIC_ENABLE | VIC_SPI0;
  VICVectAddr7 = (uint32_t)SPI0_ISR;    // address of the ISR

  SetBit(S0SPCR, SPIE);

}


static void SPI0_ISR(void)
{
  ISR_ENTRY();

  static uint8_t cnt = 0;
  LED_TOGGLE(1);

  if (bit_is_set(S0SPSR, SPIF)) {  /* transfer complete  */
    uint8_t foo = S0SPDR;
    S0SPDR = cnt;
    cnt++;
  }

  /* clear_it */
  S0SPINT = 1 << SPI0IF;


  VICVectAddr = 0x00000000; /* clear this interrupt from the VIC */
  ISR_EXIT();
}
