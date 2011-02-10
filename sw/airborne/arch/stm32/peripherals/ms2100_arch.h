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

#include <stm32/gpio.h>
#include <stm32/spi.h>

extern uint8_t ms2100_cur_axe;
extern int16_t ms2100_last_reading;

#define Ms2001Select()   GPIOC->BRR = GPIO_Pin_12
#define Ms2001Unselect() GPIOC->BSRR = GPIO_Pin_12

#define Ms2001Reset() GPIOC->BSRR = GPIO_Pin_13;
#define Ms2001Set()   GPIOC->BRR = GPIO_Pin_13

#define Ms2001HasEOC() GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_5)

#define Ms2001SendReq() {						\
    Ms2001Select();							\
    __IO uint32_t nCount = 4;for(; nCount != 0; nCount--);		\
    Ms2001Reset();							\
    ms2100_status = MS2100_SENDING_REQ;					\
    nCount = 4;for(; nCount != 0; nCount--);				\
    Ms2001Set();							\
    uint16_t ctl_byte = ((ms2100_cur_axe+1) | (MS2100_DIVISOR << 4));	\
    nCount = 20;for(; nCount != 0; nCount--);				\
    SPI_Cmd(SPI2, DISABLE);						\
    SPI_InitTypeDef SPI_InitStructure = {				\
      .SPI_Direction = SPI_Direction_2Lines_FullDuplex,			\
      .SPI_Mode = SPI_Mode_Master,					\
      .SPI_DataSize = SPI_DataSize_8b,					\
      .SPI_CPOL = SPI_CPOL_Low,						\
      .SPI_CPHA = SPI_CPHA_1Edge,					\
      .SPI_NSS = SPI_NSS_Soft,						\
      .SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_64,		\
      .SPI_FirstBit = SPI_FirstBit_MSB,					\
      .SPI_CRCPolynomial = 7						\
    };									\
    SPI_Init(SPI2, &SPI_InitStructure);					\
    SPI_I2S_ITConfig(SPI2, SPI_I2S_IT_RXNE, ENABLE);			\
    SPI_Cmd(SPI2, ENABLE);						\
    SPI_I2S_SendData(SPI2, ctl_byte);					\
  }

#define Ms2001ReadRes() {						\
    ms2100_status = MS2100_READING_RES;					\
    Ms2001Select();							\
    SPI_Cmd(SPI2, DISABLE);						\
    SPI_InitTypeDef SPI_InitStructure = {				\
      .SPI_Direction = SPI_Direction_2Lines_FullDuplex,			\
      .SPI_Mode = SPI_Mode_Master,					\
      .SPI_DataSize = SPI_DataSize_16b,					\
      .SPI_CPOL = SPI_CPOL_Low,						\
      .SPI_CPHA = SPI_CPHA_1Edge,					\
      .SPI_NSS = SPI_NSS_Soft,						\
      .SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_64,		\
      .SPI_FirstBit = SPI_FirstBit_MSB,					\
      .SPI_CRCPolynomial = 7						\
    };									\
    SPI_Init(SPI2, &SPI_InitStructure);					\
    SPI_Cmd(SPI2, ENABLE);						\
                                    \
    /* trigger 2 frames read */						\
    /* SPI2_Rx_DMA_Channel configuration ------------------------------------*/ \
    DMA_InitTypeDef  DMA_InitStructure;					\
    DMA_DeInit(DMA1_Channel4);						\
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(SPI2_BASE+0x0C); \
    DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)(&ms2100_last_reading); \
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;			\
    DMA_InitStructure.DMA_BufferSize = 1;				\
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;	\
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;		\
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord; \
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;	\
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;			\
    DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;		\
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;			\
    DMA_Init(DMA1_Channel4, &DMA_InitStructure);			\
    /* SPI2_Tx_DMA_Channel configuration ------------------------------------*/ \
    DMA_DeInit(DMA1_Channel5);						\
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(SPI2_BASE+0x0C); \
    DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)&ms2100_values;	\
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;			\
    DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;		\
    DMA_Init(DMA1_Channel5, &DMA_InitStructure);			\
                                    \
    /* Enable SPI_2 Rx request */					\
    SPI_I2S_DMACmd(SPI2, SPI_I2S_DMAReq_Rx, ENABLE);			\
    /* Enable DMA1 Channel4 */						\
    DMA_Cmd(DMA1_Channel4, ENABLE);					\
                                    \
    /* Enable SPI_2 Tx request */					\
    SPI_I2S_DMACmd(SPI2, SPI_I2S_DMAReq_Tx, ENABLE);			\
    /* Enable DMA1 Channel5 */						\
    DMA_Cmd(DMA1_Channel5, ENABLE);					\
                                    \
    /* Enable DMA1 Channel4 Transfer Complete interrupt */		\
    DMA_ITConfig(DMA1_Channel4, DMA_IT_TC, ENABLE);			\
                                        \
  }

#define Ms2001OnDmaIrq() {					\
    /*  ASSERT((ms2100_status == MS2100_READING_RES),		\
     *   DEBUG_MS2100, MS2100_ERR_SPURIOUS_DMA_IRQ);		\
     */								\
    if (abs(ms2100_last_reading) < 1000)			\
      ms2100_values[ms2100_cur_axe] = ms2100_last_reading;	\
    Ms2001Unselect();						\
    ms2100_cur_axe++;						\
    if (ms2100_cur_axe > 2) {					\
      ms2100_cur_axe = 0;					\
      ms2100_status = MS2100_DATA_AVAILABLE;			\
    }								\
    else							\
      ms2100_status = MS2100_IDLE;				\
    SPI_Cmd(SPI2, DISABLE);					\
    DMA_ITConfig(DMA1_Channel4, DMA_IT_TC, DISABLE);		\
  }

#define Ms2001OnSpiIrq() {						\
    /*  ASSERT((ms2100_status == MS2100_SENDING_REQ),			\
     *   DEBUG_MS2100, MS2100_ERR_SPURIOUS_SPI_IRQ);			\
     */									\
    /* read unused control byte reply */				\
    uint8_t foo __attribute__ ((unused)) = SPI_I2S_ReceiveData(SPI2);	\
    Ms2001Unselect();							\
    ms2100_status = MS2100_WAITING_EOC;					\
    SPI_Cmd(SPI2, DISABLE);						\
    SPI_I2S_ITConfig(SPI2, SPI_I2S_IT_RXNE, DISABLE);			\
  }

#endif /* MS2100_ARCH_H */
