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

#include <stm32/rcc.h>
#include <stm32/spi.h>
#include <stm32/exti.h>
#include <stm32/misc.h>
#include <stm32/dma.h>

uint8_t ms2100_cur_axe;
int16_t ms2100_last_reading; // can't write in place because that stupid beast
                             // stips stupid values once in a while that I need
                             // to filter - high time we get rid of this crap hardware
                             // and no, I checked with the logic analyzer, timing are
                             // within specs

void ms2100_arch_init( void ) {

  ms2100_cur_axe = 0;

  /* set mag SS and reset as output and assert them (SS on PC12  reset on PC13) ----*/
  Ms2001Unselect();
  Ms2001Set();

  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
  GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOC, &GPIO_InitStructure);

  /* configure data ready on PB5 */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

#ifdef MS2100_HANDLES_DMA_IRQ
 /* Enable DMA1 channel4 IRQ Channel */
  NVIC_InitTypeDef NVIC_init_structure_dma = {
    .NVIC_IRQChannel = DMA1_Channel4_IRQn,
    .NVIC_IRQChannelPreemptionPriority = 0,
    .NVIC_IRQChannelSubPriority = 0,
    .NVIC_IRQChannelCmd = ENABLE
  };
  NVIC_Init(&NVIC_init_structure_dma);
#endif /* MS2100_HANDLES_DMA_IRQ */

#ifdef MS2100_HANDLES_SPI_IRQ
  NVIC_InitTypeDef NVIC_init_structure_spi = {
    .NVIC_IRQChannel = SPI2_IRQn,
    .NVIC_IRQChannelPreemptionPriority = 0,
    .NVIC_IRQChannelSubPriority = 1,
    .NVIC_IRQChannelCmd = ENABLE
  };
  NVIC_Init(&NVIC_init_structure_spi);
#endif /* MS2100_HANDLES_SPI_IRQ */

}

#ifdef MS2100_HANDLES_SPI_IRQ
void spi2_irq_handler(void) {
  Ms2001OnSpiIrq();
}
#endif


#ifdef MS2100_HANDLES_DMA_IRQ
void dma1_c4_irq_handler(void) {
  Ms2001OnDmaIrq();
}
#endif /* MS2100_HANDLES_DMA_IRQ */
