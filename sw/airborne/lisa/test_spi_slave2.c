/*
 * $Id$
 *
 * Copyright (C) 2009 Antoine Drouin <poinix@gmail.com>
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

#include <stm32/flash.h>
#include <stm32/misc.h>
#include <stm32/spi.h>
#include <stm32/dma.h>

#include BOARD_CONFIG
#include "mcu.h"
#include "mcu_periph/sys_time.h"
#include "subsystems/datalink/downlink.h"
#include "fms/fms_autopilot_msg.h"
#include "led.h"

static inline void main_init( void );
static inline void main_periodic_task( void );
static inline void main_event_task( void );

#define USE_DMA 1

#define BufferSize       sizeof(struct AutopilotMessageFoo)
uint8_t SPI_SLAVE_Buffer_Rx[BufferSize];
uint8_t SPI_SLAVE_Buffer_Tx[BufferSize];
static inline void main_spi_slave_init(void);
static void main_setup_dma(void);
static void main_enable_dma(void);

int main(void) {
  main_init();

  while(1) {
    if (sys_time_check_and_ack_timer(0))
      main_periodic_task();
    main_event_task();
  }

  return 0;
}


static inline void main_init( void ) {
  mcu_init();
  sys_time_register_timer((1./PERIODIC_FREQUENCY), NULL);
  main_spi_slave_init();
}

static inline void main_periodic_task( void ) {
  RunOnceEvery(10,
	       {
		 DOWNLINK_SEND_BOOT(DefaultChannel, DefaultDevice, &sys_time.nb_sec);
		 LED_PERIODIC();
	       });

}

static inline void main_event_task( void ) {

#ifdef USE_DMA
  if (DMA_GetFlagStatus(DMA1_FLAG_TC2)) {
    LED_TOGGLE(3);
    RunOnceEvery(10, {DOWNLINK_SEND_DEBUG_MCU_LINK(DefaultChannel, DefaultDevice, &SPI_SLAVE_Buffer_Rx[0],
						    &SPI_SLAVE_Buffer_Rx[1], &SPI_SLAVE_Buffer_Rx[2]);});
    memcpy(SPI_SLAVE_Buffer_Tx, SPI_SLAVE_Buffer_Rx, BufferSize);
    main_setup_dma();
    main_enable_dma();
  }
#endif

}


static inline void main_spi_slave_init( void ) {

#ifdef USE_DMA
  /* Enable SPI_1 DMA clock ---------------------------------------------------*/
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
#endif

  /* Enable SPI1 Periph clock -------------------------------------------------*/
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);

  /* Configure GPIOs: NSS, SCK, MISO and MOSI  --------------------------------*/
  GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_Init(GPIOA, &GPIO_InitStructure);


  /* SPI_SLAVE configuration --------------------------------------------------*/
  SPI_InitTypeDef SPI_InitStructure;
  SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
  SPI_InitStructure.SPI_Mode = SPI_Mode_Slave;
  SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
  SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
  SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;
  SPI_InitStructure.SPI_NSS = SPI_NSS_Hard;
  //  SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
  SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_4;
  SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
  SPI_InitStructure.SPI_CRCPolynomial = 7;
  SPI_Init(SPI1, &SPI_InitStructure);

  /* Enable SPI_SLAVE */
  SPI_Cmd(SPI1, ENABLE);

#ifdef USE_DMA
  /* SPI_SLAVE_Rx_DMA_Channel configuration ------------------------------------*/
  main_setup_dma();
  main_enable_dma();
#endif


}


static void main_setup_dma(void) {
  /* SPI_SLAVE_Rx_DMA_Channel configuration ------------------------------------*/
  DMA_InitTypeDef  DMA_InitStructure;
  DMA_DeInit(DMA1_Channel2);
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(SPI1_BASE+0x0C);
  DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)SPI_SLAVE_Buffer_Rx;
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
  DMA_InitStructure.DMA_BufferSize = BufferSize;
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
  DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
  DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
  DMA_Init(DMA1_Channel2, &DMA_InitStructure);
  /* SPI_SLAVE_Tx_DMA_Channel configuration ------------------------------------*/
  DMA_DeInit(DMA1_Channel3);
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(SPI1_BASE+0x0C);
  DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)SPI_SLAVE_Buffer_Tx;
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
  DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;
  DMA_Init(DMA1_Channel3, &DMA_InitStructure);
}

static void main_enable_dma(void) {
  /* Enable SPI_1 Rx request */
  SPI_I2S_DMACmd(SPI1, SPI_I2S_DMAReq_Rx, ENABLE);
  /* Enable DMA1 Channel2 */
  DMA_Cmd(DMA1_Channel2, ENABLE);
  /* Enable SPI_1 Rx request */
  SPI_I2S_DMACmd(SPI1, SPI_I2S_DMAReq_Tx, ENABLE);
  /* Enable DMA1 Channel3 */
  DMA_Cmd(DMA1_Channel3, ENABLE);
}



#if 0
void spi1_irq_handler(void) {
  uint8_t foo = SPI_I2S_ReceiveData(SPI1);
  static uint8_t cnt = 0;
  SPI_I2S_SendData(SPI1, cnt);
  cnt++;
  LED_TOGGLE(4);
}
#endif


void spi1_dma_irq_handler(void) {
   LED_TOGGLE(5);
}
