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
#include <stm32/gpio.h>

#define Max1168Unselect() GPIOB->BSRR = GPIO_Pin_12
#define Max1168Select() GPIOB->BRR = GPIO_Pin_12

#define Max1168OnDmaIrq() {						\
    /*  ASSERT((max1168_status == STA_MAX1168_READING_RES),	\
     *          DEBUG_MAX_1168, MAX1168_ERR_SPURIOUS_DMA_IRQ);		\
     */									\
    Max1168Unselect();							\
    DMA_ITConfig(DMA1_Channel4, DMA_IT_TC, DISABLE);			\
    /* Disable SPI_2 Rx and TX request */				\
    SPI_I2S_DMACmd(SPI2, SPI_I2S_DMAReq_Rx, DISABLE);			\
    SPI_I2S_DMACmd(SPI2, SPI_I2S_DMAReq_Tx, DISABLE);			\
    /* Disable DMA1 Channel4 and 5 */					\
    DMA_Cmd(DMA1_Channel4, DISABLE);					\
    DMA_Cmd(DMA1_Channel5, DISABLE);					\
                                    \
    max1168_status = STA_MAX1168_DATA_AVAILABLE;			\
  }


#define Max1168ConfigureSPI() {						\
    SPI_InitTypeDef SPI_InitStructure;					\
    SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;	\
    SPI_InitStructure.SPI_Mode = SPI_Mode_Master;			\
    SPI_InitStructure.SPI_DataSize = SPI_DataSize_16b;			\
    SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;				\
    SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;			\
    SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;				\
    SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_16;	\
    SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;			\
    SPI_InitStructure.SPI_CRCPolynomial = 7;				\
    SPI_Init(SPI2, &SPI_InitStructure);					\
  }

#endif /* MAX1168_ARCH_H */
