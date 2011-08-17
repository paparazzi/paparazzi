/*
 * $Id$
 *
 * Copyright (C) 20010 Antoine Drouin <poinix@gmail.com>
 *
 * This file is part of Paparazzi.
 *
 * Paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * Paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 */

#include "subsystems/imu.h"

#include <stm32/gpio.h>
#include <stm32/rcc.h>
#include <stm32/spi.h>
#include <stm32/exti.h>
#include <stm32/misc.h>
#include <stm32/dma.h>

#define IMU_SSP_STA_IDLE           0
#define IMU_SSP_STA_BUSY_MAX1168   1
#define IMU_SSP_STA_BUSY_MS2100    2

volatile uint8_t imu_ssp_status;

void dma1_c4_irq_handler(void);
void spi2_irq_handler(void);

void imu_b2_arch_init(void) {

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
  /* Enable DMA1 channel4 IRQ Channel */
  NVIC_InitTypeDef NVIC_init_struct = {
    .NVIC_IRQChannel = DMA1_Channel4_IRQn,
    .NVIC_IRQChannelPreemptionPriority = 0,
    .NVIC_IRQChannelSubPriority = 0,
    .NVIC_IRQChannelCmd = ENABLE
  };
  NVIC_Init(&NVIC_init_struct);
  /* Enable SPI2 IRQ Channel */
  NVIC_InitTypeDef NVIC_init_structure_spi = {
    .NVIC_IRQChannel = SPI2_IRQn,
    .NVIC_IRQChannelPreemptionPriority = 0,
    .NVIC_IRQChannelSubPriority = 1,
    .NVIC_IRQChannelCmd = ENABLE
  };
  NVIC_Init(&NVIC_init_structure_spi);

  imu_ssp_status = IMU_SSP_STA_IDLE;
}

void imu_periodic(void) {
  // check ssp idle
  // ASSERT((imu_status == IMU_STA_IDLE), DEBUG_IMU, IMU_ERR_OVERUN);
  imu_ssp_status = IMU_SSP_STA_BUSY_MAX1168;
  Max1168ConfigureSPI();
  SPI_Cmd(SPI2, ENABLE);
  max1168_read();
#if IMU_B2_MAG_TYPE == IMU_B2_MAG_HMC5843
  hmc5843_periodic();
#endif
}

/* used for spi2 */
void dma1_c4_irq_handler(void) {
  switch (imu_ssp_status) {
  case IMU_SSP_STA_BUSY_MAX1168:
    Max1168OnDmaIrq();
    SPI_Cmd(SPI2, DISABLE);
#if IMU_B2_MAG_TYPE == IMU_B2_MAG_MS2100
    if (ms2100_status == MS2100_IDLE) {
      Ms2001SendReq();
      imu_ssp_status = IMU_SSP_STA_BUSY_MS2100;
    }
    else if (ms2100_status == MS2100_WAITING_EOC && Ms2001HasEOC()) {
      Ms2001ReadRes();
      imu_ssp_status = IMU_SSP_STA_BUSY_MS2100;
    }
    else
#endif
      imu_ssp_status = IMU_SSP_STA_IDLE;
    break;
  case IMU_SSP_STA_BUSY_MS2100:
#if IMU_B2_MAG_TYPE == IMU_B2_MAG_MS2100
    Ms2001OnDmaIrq();
#endif
    break;
  default:
    // POST_ERROR(DEBUG_IMU, IMU_ERR_SUPRIOUS_DMA1_C4_IRQ);
    imu_ssp_status = IMU_SSP_STA_IDLE;
  }
}


void spi2_irq_handler(void) {
#if IMU_B2_MAG_TYPE == IMU_B2_MAG_MS2100
  Ms2001OnSpiIrq();
#endif
}
