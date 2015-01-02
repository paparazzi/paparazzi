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

#include "subsystems/imu.h"

#include <stm32/gpio.h>
#include <stm32/rcc.h>
#include <stm32/spi.h>
#include <stm32/misc.h>
#include <stm32/dma.h>

static volatile uint8_t channel;
static uint8_t buf_in[4];
static uint8_t buf_out[4];

#define POWER_MODE (1 << 1 | 1)
#define SGL_DIF 1 // Single ended

#define ADS8344Unselect() GPIOB->BSRR = GPIO_Pin_12
#define ADS8344Select()   GPIOB->BRR  = GPIO_Pin_12

extern void dma1_c4_irq_handler(void);
static void ADS8344_read_channel(void);

void imu_crista_arch_init(void)
{

  channel = 0;
  /* Enable SPI2 Periph clock -------------------------------------------------*/
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);
  /* Enable SPI_2 DMA clock ---------------------------------------------------*/
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
  /* Enable PORTB GPIO clock --------------------------------------------------*/
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO , ENABLE);
  /* Configure GPIOs: SCK, MISO and MOSI  -------------------------------------*/
  GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  /* set slave select as output and assert it ( on PB12) */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  ADS8344Unselect();
  /* configure SPI after enabling it*/
  SPI_Cmd(SPI2, ENABLE);
  SPI_InitTypeDef SPI_InitStructure;
  SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
  SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
  SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
  SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
  SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
  SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
  SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_16;
  SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
  SPI_InitStructure.SPI_CRCPolynomial = 7;
  SPI_Init(SPI2, &SPI_InitStructure);

  /* Enable DMA1 channel4 IRQ Channel */
  NVIC_InitTypeDef NVIC_init_struct = {
    .NVIC_IRQChannel = DMA1_Channel4_IRQn,
    .NVIC_IRQChannelPreemptionPriority = 0,
    .NVIC_IRQChannelSubPriority = 0,
    .NVIC_IRQChannelCmd = ENABLE
  };
  NVIC_Init(&NVIC_init_struct);

}


void ADS8344_start(void)
{

  ADS8344Select();
  channel = 0;
  ADS8344_read_channel();

}

static void ADS8344_read_channel(void)
{

  // control byte
  buf_out[0] = 1 << 7 | channel << 4 | SGL_DIF << 2 | POWER_MODE;

  /* trigger 4 bytes read */
  /* SPI2_Rx_DMA_Channel configuration ------------------------------------*/
  DMA_DeInit(DMA1_Channel4);
  DMA_InitTypeDef DMA_initStructure_4 = {
    .DMA_PeripheralBaseAddr = (uint32_t)(SPI2_BASE + 0x0C),
    .DMA_MemoryBaseAddr = (uint32_t)buf_in,
    .DMA_DIR = DMA_DIR_PeripheralSRC,
    .DMA_BufferSize = 4,
    .DMA_PeripheralInc = DMA_PeripheralInc_Disable,
    .DMA_MemoryInc = DMA_MemoryInc_Enable,
    .DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte,
    .DMA_MemoryDataSize = DMA_MemoryDataSize_Byte,
    .DMA_Mode = DMA_Mode_Normal,
    .DMA_Priority = DMA_Priority_VeryHigh,
    .DMA_M2M = DMA_M2M_Disable
  };
  DMA_Init(DMA1_Channel4, &DMA_initStructure_4);

  /* SPI2_Tx_DMA_Channel configuration ------------------------------------*/
  DMA_DeInit(DMA1_Channel5);
  DMA_InitTypeDef DMA_initStructure_5 = {
    .DMA_PeripheralBaseAddr = (uint32_t)(SPI2_BASE + 0x0C),
    .DMA_MemoryBaseAddr = (uint32_t)buf_out,
    .DMA_DIR = DMA_DIR_PeripheralDST,
    .DMA_BufferSize = 4,
    .DMA_PeripheralInc = DMA_PeripheralInc_Disable,
    .DMA_MemoryInc = DMA_MemoryInc_Enable,
    .DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte,
    .DMA_MemoryDataSize = DMA_MemoryDataSize_Byte,
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

}


void dma1_c4_irq_handler(void)
{

  ADS8344_values[channel] = (buf_in[1] << 8 | buf_in[2]) << 1 | buf_in[3] >> 7;
  channel++;
  if (channel > 6) {
    ADS8344_available = TRUE;
    ADS8344Unselect();
    DMA_ITConfig(DMA1_Channel4, DMA_IT_TC, DISABLE);
    /* Disable SPI_2 Rx and TX request */
    SPI_I2S_DMACmd(SPI2, SPI_I2S_DMAReq_Rx, DISABLE);
    SPI_I2S_DMACmd(SPI2, SPI_I2S_DMAReq_Tx, DISABLE);
    /* Disable DMA1 Channel4 and 5 */
    DMA_Cmd(DMA1_Channel4, DISABLE);
    DMA_Cmd(DMA1_Channel5, DISABLE);
  } else {
    ADS8344_read_channel();
  }
}
