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

#ifndef MAX1168_ARCH_H
#define MAX1168_ARCH_H

/*
 * max1168 analog to digital converter
 * connected on spi2
 * select on PB12
 * drdy on PD2
 */
#include <libopencm3/stm32/f1/gpio.h>

#define Max1168Unselect() GPIOB_BSRR = GPIO12
#define Max1168Select() GPIOB_BRR = GPIO12

#define Max1168OnDmaIrq() {						\
    /*  ASSERT((max1168_status == STA_MAX1168_READING_RES),	\
     *          DEBUG_MAX_1168, MAX1168_ERR_SPURIOUS_DMA_IRQ);		\
     */									\
    Max1168Unselect();							\
    dma_disable_transfer_complete_interrupt(DMA1, DMA_CHANNEL4);	\
    /* Disable SPI_2 Rx and TX request */				\
    spi_disable_rx_dma(SPI2);						\
    spi_disable_tx_dma(SPI2);						\
    /* Disable DMA1 Channel4 and 5 */					\
    dma_disable_channel(DMA1, DMA_CHANNEL4);				\
    dma_disable_channel(DMA1, DMA_CHANNEL5);				\
    									\
    max1168_status = STA_MAX1168_DATA_AVAILABLE;			\
  }


#define Max1168ConfigureSPI() {						\
    spi_reset(SPI2);						        \
    spi_disable(SPI2);							\
    spi_init_master(SPI2, SPI_CR1_BAUDRATE_FPCLK_DIV_16,		\
		    SPI_CR1_CPOL_CLK_TO_0_WHEN_IDLE,			\
		    SPI_CR1_CPHA_CLK_TRANSITION_1,			\
		    SPI_CR1_DFF_16BIT,					\
		    SPI_CR1_MSBFIRST);					\
    spi_enable_software_slave_management(SPI2);				\
    spi_set_nss_high(SPI2);						\
    spi_enable(SPI2);							\
  }

#endif /* MAX1168_ARCH_H */
