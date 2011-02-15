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
#include <stm32/flash.h>
#include <stm32/misc.h>
#include <stm32/spi.h>
#include <stm32/dma.h>
#include <stm32/exti.h>

#include BOARD_CONFIG
#include "mcu.h"
#include "sys_time.h"
#include "downlink.h"
#include "peripherals/ms2100.h"

static inline void main_init( void );
static inline void main_periodic_task( void );
static inline void main_event_task( void );

static inline void main_spi2_init(void);

int main(void) {
  main_init();

  while(1) {
    if (sys_time_periodic())
      main_periodic_task();
    main_event_task();
  }

  return 0;
}


static inline void main_init( void ) {
  mcu_init();
  sys_time_init();
  ms2100_init();
  main_spi2_init();
}

static inline void main_periodic_task( void ) {
  RunOnceEvery(10,
	       {
		 DOWNLINK_SEND_BOOT(DefaultChannel, &cpu_time_sec);
		 LED_PERIODIC();
	       });

  switch(ms2100_status) {
  case MS2100_IDLE:
    Ms2001SendReq();
    break;
  case MS2100_WAITING_EOC:
    if (Ms2001HasEOC()) {
      Ms2001ReadRes();
    }
    break;
  }
}

static inline void main_event_task( void ) {
  if (ms2100_status == MS2100_DATA_AVAILABLE) {
    RunOnceEvery(10, {
	DOWNLINK_SEND_IMU_MAG_RAW(DefaultChannel,
				  &ms2100_values[0],
				  &ms2100_values[1],
				  &ms2100_values[2]);
      });
    ms2100_status = MS2100_IDLE;
  }
}

static inline void main_spi2_init( void ) {

  /* set max1168 slave select as output and assert it ( on PB12) */
  GPIOB->BSRR = GPIO_Pin_12;
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
  GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  /* Enable SPI2 Periph clock -------------------------------------------------*/
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);

  /* Configure GPIOs: SCK, MISO and MOSI  --------------------------------*/
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO , ENABLE);


  /* Enable SPI_2 DMA clock ---------------------------------------------------*/
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);



}

