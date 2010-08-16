/*
 * $Id$
 *  
 * Copyright (C) 2010 Eric Parsonage <eric@eparsonage.com>
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
 *
 */

#include <stdint.h>
#include <stm32/gpio.h>
#include <stm32/rcc.h>
#include <stm32/tim.h>

#include BOARD_CONFIG

#include "uart.h"
#include "booz/booz_radio_control.h"

/* set TIM1 to run at DELAY_TIM_FREQUENCY */ 
static void delay_init( void );
/* wait busy loop, microseconds */
static void delay_us( uint16_t uSecs );
/* wait busy loop, milliseconds */
static void delay_ms( uint16_t mSecs ); 
 
/* The longest delay with micro second granularity */
#define MAX_DELAY   INT16_MAX
/* the frequency of the delay timer */
#define DELAY_TIM_FREQUENCY 1000000
/* Number of low pulses sent to satellite receivers */ 
#define MASTER_RECEIVER_PULSES 3 
#define SLAVE_RECEIVER_PULSES 4 

/* The line that is pulled low at power up to initiate the bind process */  
#define BIND_PIN GPIO_Pin_3
#define BIND_PIN_PORT GPIOC
#define BIND_PIN_PERIPH RCC_APB2Periph_GPIOC 


/*
 * in the makefile we set RADIO_CONTROL_LINK to be Uartx
 * but in uart_hw.c the initialisation functions are 
 * defined as uartx these macros give us the glue 
 * that allows static calls at compile time 
 */

#define Uart1_init uart1_init()
#define Uart2_init uart2_init()
#define Uart3_init uart3_init()
#define Uart5_init uart5_init()

#define __MasterRcLink(dev, _x) dev##_x
#define _MasterRcLink(dev, _x)  __MasterRcLink(dev, _x)
#define MasterRcLink(_x) _MasterRcLink(RADIO_CONTROL_LINK, _x)

#define __SlaveRcLink(dev, _x) dev##_x
#define _SlaveRcLink(dev, _x)  __SlaveRcLink(dev, _x)
#define SlaveRcLink(_x) _SlaveRcLink(RADIO_CONTROL_LINK_SLAVE, _x)


/*
 * bind() init must called on powerup as the spektrum 
 * satellites can only bind immediately after power up
 * also it must be called before we call uartx_init()
 * as we leave them with their Rx pins set as outputs
 */    
void radio_control_spektrum_try_bind( void ) {

  RCC_APB2PeriphClockCmd(BIND_PIN_PERIPH , ENABLE);
  GPIO_InitTypeDef gpio;
  gpio.GPIO_Pin = BIND_PIN;
  gpio.GPIO_Mode = GPIO_Mode_IPU;
  gpio.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_Init(BIND_PIN_PORT, &gpio);

  /* exit if the BIND_PIN is high, it needs to be pulled low at startup to initiate bind */
  if (GPIO_ReadInputDataBit(BIND_PIN_PORT, BIND_PIN)) 
    return;

  /* bind initiated, initialise the delay timer */
  delay_init();

  /* initialise the uarts rx pins as  GPIOS */
  RCC_APB2PeriphClockCmd(MasterRcLink(_Periph) , ENABLE);
  /* Master receiver Rx push-pull */
  gpio.GPIO_Pin = MasterRcLink(_RxPin);
  gpio.GPIO_Mode = GPIO_Mode_Out_PP;
  gpio.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(MasterRcLink(_RxPort), &gpio);
  /* Master receiver RX line, drive high */
  GPIO_WriteBit(MasterRcLink(_RxPort), MasterRcLink(_RxPin) , Bit_SET );

#ifdef RADIO_CONTROL_LINK_SLAVE
   RCC_APB2PeriphClockCmd(SlaveRcLink(_Periph) , ENABLE);
  /* Slave receiver Rx push-pull */
  gpio.GPIO_Pin = SlaveRcLink(_RxPin);
  gpio.GPIO_Mode = GPIO_Mode_Out_PP;
  gpio.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(SlaveRcLink(_RxPort), &gpio);
  /* Slave receiver RX line, drive high */
  GPIO_WriteBit(SlaveRcLink(_RxPort), SlaveRcLink(_RxPin) , Bit_SET );
#endif

  /* We have no idea how long the window for allowing binding after  
     power up is .This works for the moment but will need revisiting */	
  delay_ms(73);

  for (int i = 0; i < MASTER_RECEIVER_PULSES ; i++) 
  {
    GPIO_WriteBit(MasterRcLink(_RxPort), MasterRcLink(_RxPin), Bit_RESET );
    delay_us(120);
    GPIO_WriteBit(MasterRcLink(_RxPort), MasterRcLink(_RxPin), Bit_SET );
    delay_us(120);
  }
  
#ifdef RADIO_CONTROL_LINK_SLAVE
  for (int i = 0; i < SLAVE_RECEIVER_PULSES; i++) 
  {
    GPIO_WriteBit(SlaveRcLink(_RxPort), SlaveRcLink(_RxPin), Bit_RESET );
    delay_us(120);
    GPIO_WriteBit(SlaveRcLink(_RxPort), SlaveRcLink(_RxPin), Bit_SET );
    delay_us(120);
  }
#endif /* RADIO_CONTROL_LINK_SLAVE */
}


/*
 *\Functions to implement busy wait loops with micro second granularity
 *
 */

/* set TIM1 to run at DELAY_TIM_FREQUENCY */ 
static void delay_init( void ) {
  /* Enable timer clock */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
  /* Time base configuration */
  TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
  TIM_TimeBaseStructInit(&TIM_TimeBaseStructure); 
  TIM_TimeBaseStructure.TIM_Prescaler = (AHB_CLK / DELAY_TIM_FREQUENCY) - 1;
  TIM_TimeBaseStructure.TIM_Period = UINT16_MAX; 
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);
  
 /* Enable counter */
  TIM_Cmd(TIM1, ENABLE);
}

/* wait busy loop, microseconds */
static void delay_us( uint16_t uSecs ) {
  uint16_t start = TIM1->CNT;
  /* use 16 bit count wrap around */
  while((uint16_t)(TIM1->CNT - start) <= uSecs);
}

/* wait busy loop, milliseconds */
static void delay_ms( uint16_t mSecs ) {
  for(int i = 0; i < mSecs; i++) {
    delay_us(DELAY_TIM_FREQUENCY / 1000);
  }
}



