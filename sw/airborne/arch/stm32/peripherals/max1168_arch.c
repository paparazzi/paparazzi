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

#include <stm32/rcc.h>
#include <stm32/spi.h>
#include <stm32/exti.h>
#include <stm32/misc.h>
#include <stm32/dma.h>


/* I can't use GPIOD as it's already defined in some system header */
#define __DRDY_PORT(dev, _x) _x##dev
#define _DRDY_PORT(dev, _x)  __DRDY_PORT(dev, _x)
#define DRDY_PORT(_x) _DRDY_PORT(MAX_1168_DRDY_PORT, _x)

#define __DRDY_PORT_SOURCE(dev, _x) _x##dev
#define _DRDY_PORT_SOURCE(dev, _x)  __DRDY_PORT_SOURCE(dev, _x)
#define DRDY_PORT_SOURCE(_x) _DRDY_PORT_SOURCE(MAX_1168_DRDY_PORT_SOURCE, _x)

void exti2_irq_handler(void);

void max1168_arch_init( void ) {

  /* set slave select as output and assert it ( on PB12) */
  Max1168Unselect();
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
  GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  /* configure external interrupt exti2 on PD2( data ready ) v1.0*/
  /*                                       PB2( data ready ) v1.1*/
  //  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD | RCC_APB2Periph_AFIO, ENABLE);
  RCC_APB2PeriphClockCmd(DRDY_PORT(RCC_APB2Periph) | RCC_APB2Periph_AFIO, ENABLE);
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOD, &GPIO_InitStructure);

  EXTI_InitTypeDef EXTI_InitStructure;
  //  GPIO_EXTILineConfig(GPIO_PortSourceGPIOD, GPIO_PinSource2);
  GPIO_EXTILineConfig(DRDY_PORT_SOURCE(GPIO_), GPIO_PinSource2);
  EXTI_InitStructure.EXTI_Line = EXTI_Line2;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);

  NVIC_InitTypeDef NVIC_InitStructure;
  NVIC_InitStructure.NVIC_IRQChannel = EXTI2_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0F;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0F;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;

  NVIC_Init(&NVIC_InitStructure);

#ifdef MAX1168_HANDLES_DMA_IRQ
  /* Enable DMA1 channel4 IRQ Channel */
  NVIC_InitTypeDef NVIC_init_struct = {
    .NVIC_IRQChannel = DMA1_Channel4_IRQn,
    .NVIC_IRQChannelPreemptionPriority = 0,
    .NVIC_IRQChannelSubPriority = 0,
    .NVIC_IRQChannelCmd = ENABLE
  };
  NVIC_Init(&NVIC_init_struct);
#endif /* MAX1168_HANDLES_DMA_IRQ */
}

void max1168_read( void ) {
  /*  ASSERT((max1168_status == STA_MAX1168_IDLE),	\
   *   DEBUG_MAX_1168, MAX1168_ERR_READ_OVERUN);
   */
  /* select max1168 */
  Max1168Select();

  /* write control byte - wait EOC on extint */
  /* use internal reference and clock, sequentially scan channels 0-7 */
  const uint16_t ctl_byte = (1 << 0 | 1 << 3 | 7 << 5) << 8;
  SPI_I2S_SendData(SPI2, ctl_byte);
  max1168_status = STA_MAX1168_SENDING_REQ;

}

void exti2_irq_handler(void) {

  /*  ASSERT((max1168_status == STA_MAX1168_SENDING_REQ),	\
   *     DEBUG_MAX_1168, MAX1168_ERR_SPURIOUS_EOC);
   */

  /* clear EXTI */
  if(EXTI_GetITStatus(EXTI_Line2) != RESET)
    EXTI_ClearITPendingBit(EXTI_Line2);

  /* read control byte FIXME: is this needed ?, yes*/
   uint16_t foo __attribute__ ((unused)) = SPI_I2S_ReceiveData(SPI2);

  /* trigger 8 frames read */
  /* SPI2_Rx_DMA_Channel configuration ------------------------------------*/
  DMA_DeInit(DMA1_Channel4);
  DMA_InitTypeDef DMA_initStructure_4 = {
    .DMA_PeripheralBaseAddr = (uint32_t)(SPI2_BASE+0x0C),
    .DMA_MemoryBaseAddr = (uint32_t)max1168_values,
    .DMA_DIR = DMA_DIR_PeripheralSRC,
    .DMA_BufferSize = MAX1168_NB_CHAN,
    .DMA_PeripheralInc = DMA_PeripheralInc_Disable,
    .DMA_MemoryInc = DMA_MemoryInc_Enable,
    .DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord,
    .DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord,
    .DMA_Mode = DMA_Mode_Normal,
    .DMA_Priority = DMA_Priority_VeryHigh,
    .DMA_M2M = DMA_M2M_Disable
  };
  DMA_Init(DMA1_Channel4, &DMA_initStructure_4);

  /* SPI2_Tx_DMA_Channel configuration ------------------------------------*/
  DMA_DeInit(DMA1_Channel5);
  DMA_InitTypeDef DMA_initStructure_5 = {
    .DMA_PeripheralBaseAddr = (uint32_t)(SPI2_BASE+0x0C),
    .DMA_MemoryBaseAddr = (uint32_t)max1168_values,
    .DMA_DIR = DMA_DIR_PeripheralDST,
    .DMA_BufferSize = MAX1168_NB_CHAN,
    .DMA_PeripheralInc = DMA_PeripheralInc_Disable,
    .DMA_MemoryInc = DMA_MemoryInc_Enable,
    .DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord,
    .DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord,
    .DMA_Mode = DMA_Mode_Normal,
    .DMA_Priority = DMA_Priority_Medium,
    .DMA_M2M = DMA_M2M_Disable
  };
  DMA_Init(DMA1_Channel5, &DMA_initStructure_5);

  /* Enable SPI_2 Rx request */
  SPI_I2S_DMACmd(SPI2, SPI_I2S_DMAReq_Rx, ENABLE);
  /* Enable DMA1 Channel4 */
  DMA_Cmd(DMA1_Channel4, ENABLE);

  /* Enable SPI_2 Tx request */
  SPI_I2S_DMACmd(SPI2, SPI_I2S_DMAReq_Tx, ENABLE);
  /* Enable DMA1 Channel5 */
  DMA_Cmd(DMA1_Channel5, ENABLE);

  /* Enable DMA1 Channel4 Transfer Complete interrupt */
  DMA_ITConfig(DMA1_Channel4, DMA_IT_TC, ENABLE);

  max1168_status = STA_MAX1168_READING_RES;
}

#ifdef MAX1168_HANDLES_DMA_IRQ
void dma1_c4_irq_handler(void) {
  Max1168OnDmaIrq();
}
#endif /*MAX1168_HANDLES_DMA_IRQ */
