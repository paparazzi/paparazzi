/*
 * $Id$
 *
 * Copyright (C) 2010 The Paparazzi Team
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
#include <stm32/gpio.h>
#include <stm32/rcc.h>

#include BOARD_CONFIG
#include "mcu.h"
#include "mcu_periph/sys_time.h"
#include "led.h"

#define A_PERIPH   RCC_APB2Periph_GPIOA
#define A_RX_PIN   GPIO_Pin_10
#define A_RX_PORT  GPIOA
#define A_TX_PIN   GPIO_Pin_9
#define A_TX_PORT  GPIOA

#define B_PERIPH   RCC_APB2Periph_GPIOA
#define B_RX_PIN   GPIO_Pin_3
#define B_RX_PORT  GPIOA
#define B_TX_PIN   GPIO_Pin_2
#define B_TX_PORT  GPIOA

static inline void main_periodic( void );
static inline void main_event( void );

void Delay(__IO uint32_t nCount) {
  for(; nCount != 0; nCount--);
}

int main(void) {

  mcu_init();
  sys_time_register_timer((1./PERIODIC_FREQUENCY), NULL);

  LED_INIT(2);
  LED_OFF(2);

  /* init RCC */
  RCC_APB2PeriphClockCmd(A_PERIPH , ENABLE);
  //  RCC_APB2PeriphClockCmd(B_PERIPH , ENABLE);
  //  GPIO_DeInit(A_RX_PORT);
  /* Init GPIO for rx pins */
  GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
  //  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Pin = A_RX_PIN;
  GPIO_Init(A_RX_PORT, &GPIO_InitStructure);
  GPIO_InitStructure.GPIO_Pin = B_RX_PIN;
  GPIO_Init(B_RX_PORT, &GPIO_InitStructure);
  /* Init GPIO for tx pins */
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Pin = A_TX_PIN;
  GPIO_Init(A_TX_PORT, &GPIO_InitStructure);
  GPIO_InitStructure.GPIO_Pin = B_TX_PIN;
  GPIO_Init(B_TX_PORT, &GPIO_InitStructure);

  A_TX_PORT->BRR  = A_TX_PIN;

  /* */
  while (1) {
    if (sys_time_check_and_ack_timer(0))
      main_periodic();
    main_event();
  }

  return 0;
}



static inline void main_periodic( void ) {
  LED_PERIODIC();
}

static inline void main_event( void ) {
  //  Delay(2000);
  static uint8_t foo = 0;
  foo++;

#if 0
  if (!(foo%2))
    GPIO_WriteBit(B_TX_PORT,  B_TX_PIN, Bit_SET);
  else
    GPIO_WriteBit(B_TX_PORT,  B_TX_PIN, Bit_RESET);
#endif

#if 0
  if (!(foo%2))
    A_TX_PORT->BRR  = A_TX_PIN;
  else
    A_TX_PORT->BSRR  = A_TX_PIN;
#endif

#if 1
  GPIO_WriteBit(A_TX_PORT, A_TX_PIN, GPIO_ReadInputDataBit(B_RX_PORT, B_RX_PIN) );
#endif
  if (GPIO_ReadInputDataBit(A_RX_PORT, A_RX_PIN)) {
    GPIO_WriteBit(B_TX_PORT, B_TX_PIN,  Bit_SET);
    LED_ON(2);
  }
  else {
    GPIO_WriteBit(B_TX_PORT, B_TX_PIN,  Bit_RESET);
    LED_OFF(2);
  }


}
