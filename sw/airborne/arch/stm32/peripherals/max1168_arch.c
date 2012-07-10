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
#include "peripherals/max1168.h"

#include <libopencm3/stm32/f1/rcc.h>
#include <libopencm3/stm32/f1/gpio.h>
#include <libopencm3/stm32/spi.h>
#include <libopencm3/stm32/exti.h>
#include <libopencm3/stm32/f1/dma.h>
#include <libopencm3/stm32/nvic.h>


/* I can't use GPIOD as it's already defined in some system header */
#define __DRDY_PORT(dev, _x) dev##_x
#define _DRDY_PORT(dev, _x)  __DRDY_PORT(dev, _x)
#define DRDY_PORT(_x) _DRDY_PORT(MAX_1168_DRDY_PORT, _x)

void exti2_isr(void);

void max1168_arch_init( void ) {

  /* set slave select as output and assert it ( on PB12) */
  Max1168Unselect();
  rcc_peripheral_enable_clock(&RCC_APB2ENR, RCC_APB2ENR_IOPBEN);
  gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_50_MHZ,
	  GPIO_CNF_OUTPUT_PUSHPULL, GPIO12);

  /* configure external interrupt exti2 on PD2( data ready ) v1.0*/
  /*                                       PB2( data ready ) v1.1*/
  rcc_peripheral_enable_clock(&RCC_APB2ENR, RCC_APB2ENR_IOPBEN);
  gpio_set_mode(GPIOB, GPIO_MODE_INPUT,
	  GPIO_CNF_INPUT_FLOAT, GPIO2);

  exti_select_source(EXTI2, GPIOB);
  exti_set_trigger(EXTI2, EXTI_TRIGGER_FALLING);
  exti_enable_request(EXTI2);

  nvic_set_priority(NVIC_EXTI2_IRQ, 0xF);
  nvic_enable_irq(NVIC_EXTI2_IRQ);

#ifdef MAX1168_HANDLES_DMA_IRQ
  /* Enable DMA1 channel4 IRQ Channel */
  nvic_set_priority(NVIC_DMA1_CHANNEL4_IRQ, 0);
  nvic_enable_irq(NVIC_DMA1_CHANNEL4_IRQ);
#endif /* MAX1168_HANDLES_DMA_IRQ */
}

void max1168_read( void ) {

  /* select max1168 */
  Max1168Select();

  /* write control byte - wait EOC on extint */
  /* use internal reference and clock, sequentially scan channels 0-7 */
  const uint16_t ctl_byte = (1 << 0 | 1 << 3 | 7 << 5) << 8;
  spi_write(SPI2, ctl_byte);
  max1168_status = STA_MAX1168_SENDING_REQ;

}

void exti2_isr(void) {

  /* clear EXTI */
  exti_reset_request(EXTI2);

  /* read control byte FIXME: is this needed ?, yes*/
  uint16_t foo __attribute__ ((unused)) = spi_read(SPI2);

  /* trigger 8 frames read */
  // SPI2_Rx_DMA_Channel configuration ------------------------------------
  dma_channel_reset(DMA1, DMA_CHANNEL4);
  dma_set_peripheral_address(DMA1, DMA_CHANNEL4, (u32)&SPI2_DR);
  dma_set_memory_address(DMA1, DMA_CHANNEL4, (uint32_t)max1168_values);
  dma_set_number_of_data(DMA1, DMA_CHANNEL4, MAX1168_NB_CHAN);
  dma_set_read_from_peripheral(DMA1, DMA_CHANNEL4);
  //dma_disable_peripheral_increment_mode(DMA1, DMA_CHANNEL4);
  dma_enable_memory_increment_mode(DMA1, DMA_CHANNEL4);
  dma_set_peripheral_size(DMA1, DMA_CHANNEL4, DMA_CCR_PSIZE_16BIT);
  dma_set_memory_size(DMA1, DMA_CHANNEL4, DMA_CCR_MSIZE_16BIT);
  //dma_set_mode(DMA1, DMA_CHANNEL4, DMA_???_NORMAL);
  dma_set_priority(DMA1, DMA_CHANNEL4, DMA_CCR_PL_VERY_HIGH);

  // SPI2_Tx_DMA_Channel configuration ------------------------------------
  dma_channel_reset(DMA1, DMA_CHANNEL5);
  dma_set_peripheral_address(DMA1, DMA_CHANNEL5, (u32)&SPI2_DR);
  dma_set_memory_address(DMA1, DMA_CHANNEL5, (uint32_t)max1168_values);
  dma_set_number_of_data(DMA1, DMA_CHANNEL5, MAX1168_NB_CHAN);
  dma_set_read_from_memory(DMA1, DMA_CHANNEL5);
  //dma_disable_peripheral_increment_mode(DMA1, DMA_CHANNEL5);
  dma_enable_memory_increment_mode(DMA1, DMA_CHANNEL5);
  dma_set_peripheral_size(DMA1, DMA_CHANNEL5, DMA_CCR_PSIZE_16BIT);
  dma_set_memory_size(DMA1, DMA_CHANNEL5, DMA_CCR_MSIZE_16BIT);
  //dma_set_mode(DMA1, DMA_CHANNEL5, DMA_???_NORMAL);
  dma_set_priority(DMA1, DMA_CHANNEL5, DMA_CCR_PL_MEDIUM);

  // Enable DMA1 Channel4
  dma_enable_channel(DMA1, DMA_CHANNEL4);
  // Enable SPI_2 Rx request
  spi_enable_rx_dma(SPI2);

  // Enable DMA1 Channel5
  dma_enable_channel(DMA1, DMA_CHANNEL5);
  // Enable SPI_2 Tx request
  spi_enable_tx_dma(SPI2);

  // Enable DMA1 Channel4 Transfer Complete interrupt
  dma_enable_transfer_complete_interrupt(DMA1, DMA_CHANNEL4);

  max1168_status = STA_MAX1168_READING_RES;
}

#ifdef MAX1168_HANDLES_DMA_IRQ
void dma1_channel4_isr(void) {
  Max1168OnDmaIrq();
}
#endif /*MAX1168_HANDLES_DMA_IRQ */
