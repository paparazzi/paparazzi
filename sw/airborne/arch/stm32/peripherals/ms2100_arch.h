#ifndef MS2100_ARCH_H
#define MS2100_ARCH_H

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

#include <libopencm3/stm32/f1/gpio.h>
#include <libopencm3/stm32/spi.h>

extern uint8_t ms2100_cur_axe;
extern int16_t ms2100_last_reading;

#define Ms2100Select()   GPIOC_BRR = GPIO12
#define Ms2100Unselect() GPIOC_BSRR = GPIO12

#define Ms2100Reset() GPIOC_BSRR = GPIO13;
#define Ms2100Set()   GPIOC_BRR = GPIO13

#define Ms2100HasEOC() (gpio_get(GPIOB, GPIO5) != 0)

#define Ms2100SendReq() {						\
    Ms2100Select();							\
    volatile uint32_t nCount = 4;for(; nCount != 0; nCount--);		\
    Ms2100Reset();							\
    ms2100_status = MS2100_SENDING_REQ;					\
    nCount = 4;for(; nCount != 0; nCount--);				\
    Ms2100Set();							\
    uint16_t ctl_byte = ((ms2100_cur_axe+1) | (MS2100_DIVISOR << 4));	\
    nCount = 20;for(; nCount != 0; nCount--);				\
    spi_disable(SPI2);							\
    spi_reset(SPI2);							\
    spi_init_master(SPI2, SPI_CR1_BAUDRATE_FPCLK_DIV_64,		\
		    SPI_CR1_CPOL_CLK_TO_0_WHEN_IDLE,			\
		    SPI_CR1_CPHA_CLK_TRANSITION_1,			\
		    SPI_CR1_DFF_8BIT,					\
		    SPI_CR1_MSBFIRST);					\
    spi_enable_software_slave_management(SPI2);				\
    spi_set_nss_high(SPI2);						\
    spi_enable_rx_buffer_not_empty_interrupt(SPI2);			\
    spi_enable(SPI2);							\
    SPI_DR(SPI2) = ctl_byte;						\
  }

#define Ms2100ReadRes() {						\
    ms2100_status = MS2100_READING_RES;					\
    Ms2100Select();							\
    spi_disable(SPI2);							\
    spi_reset(SPI2);							\
    spi_init_master(SPI2, SPI_CR1_BAUDRATE_FPCLK_DIV_64,		\
		    SPI_CR1_CPOL_CLK_TO_0_WHEN_IDLE,			\
		    SPI_CR1_CPHA_CLK_TRANSITION_1,			\
		    SPI_CR1_DFF_16BIT,					\
		    SPI_CR1_MSBFIRST);					\
    spi_enable_software_slave_management(SPI2);				\
    spi_set_nss_high(SPI2);						\
    spi_enable(SPI2);							\
    									\
    /* trigger 2 frames read */						\
    dma_channel_reset(DMA1, DMA_CHANNEL4);				\
    dma_set_peripheral_address(DMA1, DMA_CHANNEL4, (u32)&SPI2_DR);	\
    dma_set_memory_address(DMA1, DMA_CHANNEL4, (uint32_t)(&ms2100_last_reading)); \
    dma_set_number_of_data(DMA1, DMA_CHANNEL4, 1);			\
    dma_set_read_from_peripheral(DMA1, DMA_CHANNEL4);			\
    /*dma_disable_peripheral_increment_mode(DMA1, DMA_CHANNEL4); */	\
    dma_enable_memory_increment_mode(DMA1, DMA_CHANNEL4);		\
    dma_set_peripheral_size(DMA1, DMA_CHANNEL4, DMA_CCR_PSIZE_16BIT);	\
    dma_set_memory_size(DMA1, DMA_CHANNEL4, DMA_CCR_MSIZE_16BIT);	\
    /*dma_set_mode(DMA1, DMA_CHANNEL4, DMA_???_NORMAL); */		\
    dma_set_priority(DMA1, DMA_CHANNEL4, DMA_CCR_PL_VERY_HIGH);		\
									\
    /* SPI2_Tx_DMA_Channel configuration ----------------------------*/ \
    dma_channel_reset(DMA1, DMA_CHANNEL5);				\
    dma_set_peripheral_address(DMA1, DMA_CHANNEL5, (u32)&SPI2_DR);	\
    dma_set_memory_address(DMA1, DMA_CHANNEL5, (uint32_t)(&ms2100_values)); \
    dma_set_number_of_data(DMA1, DMA_CHANNEL5, 1);			\
    dma_set_read_from_memory(DMA1, DMA_CHANNEL5);			\
    /*dma_disable_peripheral_increment_mode(DMA1, DMA_CHANNEL5); */	\
    dma_enable_memory_increment_mode(DMA1, DMA_CHANNEL5);		\
    dma_set_peripheral_size(DMA1, DMA_CHANNEL5, DMA_CCR_PSIZE_16BIT);	\
    dma_set_memory_size(DMA1, DMA_CHANNEL5, DMA_CCR_MSIZE_16BIT);	\
    /*dma_set_mode(DMA1, DMA_CHANNEL5, DMA_???_NORMAL); */		\
    dma_set_priority(DMA1, DMA_CHANNEL5, DMA_CCR_PL_MEDIUM);		\
									\
    /* Enable DMA1 Channel4 */						\
    dma_enable_channel(DMA1, DMA_CHANNEL4);				\
    /* Enable SPI_2 Rx request */					\
    spi_enable_rx_dma(SPI2);						\
									\
    /* Enable DMA1 Channel5 */						\
    dma_enable_channel(DMA1, DMA_CHANNEL5);				\
    /* Enable SPI_2 Tx request */					\
    spi_enable_tx_dma(SPI2);						\
									\
    /* Enable DMA1 Channel4 Transfer Complete interrupt */		\
    dma_enable_transfer_complete_interrupt(DMA1, DMA_CHANNEL4);		\
  }

#define Ms2100OnDmaIrq() {					\
    if (abs(ms2100_last_reading) < 1000)			\
      ms2100_values[ms2100_cur_axe] = ms2100_last_reading;	\
    Ms2100Unselect();						\
    ms2100_cur_axe++;						\
    if (ms2100_cur_axe > 2) {					\
      ms2100_cur_axe = 0;					\
      ms2100_status = MS2100_DATA_AVAILABLE;			\
    }								\
    else							\
      ms2100_status = MS2100_IDLE;				\
    spi_disable(SPI2);							\
    dma_disable_transfer_complete_interrupt(DMA1, DMA_CHANNEL4);	\
  }

#define Ms2100OnSpiIrq() {						\
    /* read unused control byte reply */				\
    uint8_t foo __attribute__ ((unused)) = SPI_DR(SPI2);		\
    Ms2100Unselect();							\
    ms2100_status = MS2100_WAITING_EOC;					\
    spi_disable(SPI2);							\
    spi_disable_rx_buffer_not_empty_interrupt(SPI2);			\
  }

#endif /* MS2100_ARCH_H */
