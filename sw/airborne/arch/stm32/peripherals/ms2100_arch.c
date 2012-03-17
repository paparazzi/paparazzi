/*
 * $Id$
 *
 * Copyright (C) 2010 Antoine Drouin <poinix@gmail.com>
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

#include "peripherals/ms2100.h"

#include <libopencm3/stm32/f1/rcc.h>
#include <libopencm3/stm32/spi.h>
#include <libopencm3/stm32/exti.h>
#include <libopencm3/stm32/f1/dma.h>
#include <libopencm3/stm32/nvic.h>

uint8_t ms2100_cur_axe;
int16_t ms2100_last_reading; // can't write in place because that stupid beast
                             // stips stupid values once in a while that I need
                             // to filter - high time we get rid of this crap hardware
                             // and no, I checked with the logic analyzer, timing are
                             // within specs

void ms2100_arch_init( void ) {

  ms2100_cur_axe = 0;

  /* set mag SS and reset as output and assert them (SS on PC12  reset on PC13) ----*/
  Ms2100Unselect();
  Ms2100Set();

  /* Configure clocks */
  rcc_peripheral_enable_clock(&RCC_APB2ENR, RCC_APB2ENR_IOPCEN | RCC_APB2ENR_IOPBEN);

  /* Configure chip select */
  gpio_set_mode(GPIOC, GPIO_MODE_OUTPUT_50_MHZ,
	        GPIO_CNF_OUTPUT_PUSHPULL, GPIO12 | GPIO13);

  /* configure data ready on PB5 */
  gpio_set_mode(GPIOB, GPIO_MODE_INPUT,
	        GPIO_CNF_INPUT_FLOAT, GPIO5);

#ifdef MS2100_HANDLES_DMA_IRQ
  /* Enable DMA1 channel4 IRQ Channel */
  nvic_set_priority(NVIC_DMA1_CHANNEL4_IRQ, 0);
  nvic_enable_irq(NVIC_DMA1_CHANNEL4_IRQ);
#endif /* MS2100_HANDLES_DMA_IRQ */

#ifdef MS2100_HANDLES_SPI_IRQ
  nvic_set_priority(NVIC_SPI2_IRQ, 1);
  nvic_enable_irq(NVIC_SPI2_IRQ);
#endif /* MS2100_HANDLES_SPI_IRQ */

}

#ifdef MS2100_HANDLES_SPI_IRQ
void spi2_irq_handler(void) {
  Ms2100OnSpiIrq();
}
#endif


#ifdef MS2100_HANDLES_DMA_IRQ
void dma1_c4_irq_handler(void) {
  Ms2100OnDmaIrq();
}
#endif /* MS2100_HANDLES_DMA_IRQ */
