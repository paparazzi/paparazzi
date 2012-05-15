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

#include BOARD_CONFIG
#include "mcu.h"
#include "mcu_periph/sys_time.h"
#include "subsystems/datalink/downlink.h"
#include "led.h"

static inline void main_init( void );
static inline void main_periodic_task( void );
static inline void main_event_task( void );

static inline void main_spi_slave_init( void );


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
  LED_INIT(3);
  LED_OFF(3);
  sys_time_register_timer((1./PERIODIC_FREQUENCY), NULL);
  main_spi_slave_init();
}

static inline void main_periodic_task( void ) {
#if 0
  static uint8_t foo = FALSE;
  if (foo)
    GPIOC->BRR = GPIO_Pin_4;
  else
    GPIOC->BSRR = GPIO_Pin_4;
  foo = !foo;
#endif
  RunOnceEvery(10, {DOWNLINK_SEND_BOOT(DefaultChannel, DefaultDevice, &sys_time.nb_sec);});
  LED_PERIODIC();
}

static inline void main_event_task( void ) {

}


static inline void main_spi_slave_init( void ) {

  /* Enable SPI1 Periph clock -------------------------------------------------*/
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);

  /* Configure GPIOs: SCK, MISO and MOSI  -------------------------------------*/
  GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  /* Configure the nested vectored interrupt controller. ----------------------*/
  NVIC_InitTypeDef NVIC_InitStructure;
  /* 1 bit for pre-emption priority, 3 bits for subpriority */
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
  NVIC_InitStructure.NVIC_IRQChannel = SPI1_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_Init(&NVIC_InitStructure);

  /* SPI_SLAVE configuration --------------------------------------------------*/
  SPI_InitTypeDef SPI_InitStructure;
  //  SPI_InitStructure.SPI_Direction = SPI_Direction_1Line_Rx;
  SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
  SPI_InitStructure.SPI_Mode = SPI_Mode_Slave;
  SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
  SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
  SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;
  SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
  SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_4;
  SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
  SPI_InitStructure.SPI_CRCPolynomial = 7;
  SPI_Init(SPI1, &SPI_InitStructure);

  /* Enable SPI_SLAVE RXNE interrupt */
  SPI_I2S_ITConfig(SPI1, SPI_I2S_IT_RXNE, ENABLE);
  /* Enable SPI_SLAVE */
  SPI_Cmd(SPI1, ENABLE);

#if 0
  /* configure DATA_READY on PC4 */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(GPIOC, &GPIO_InitStructure);
#endif
}

void spi1_irq_handler(void) {
  uint8_t foo = SPI_I2S_ReceiveData(SPI1);
  static uint8_t cnt = 0;
  SPI_I2S_SendData(SPI1, cnt);
  cnt++;
  LED_TOGGLE(3);
  DOWNLINK_SEND_DEBUG_MCU_LINK(DefaultChannel, DefaultDevice, &foo, &foo, &cnt);


}

