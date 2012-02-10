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

// addr 4
// J6 = SCL orange
// J7 = SDA

#include <stm32/flash.h>
#include <stm32/misc.h>
#include <stm32/i2c.h>

#include BOARD_CONFIG
#include "mcu.h"
#include "mcu_periph/sys_time.h"
#include "led.h"

static inline void main_init( void );
static inline void main_periodic_task( void );
static inline void main_event_task( void );

static inline void main_i2c_init( void );
static inline void main_test_send( void );
static void test_gpios(void);

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
  main_i2c_init();
  test_gpios();
}

static inline void main_periodic_task( void ) {

  main_test_send();
  LED_PERIODIC();
}

static inline void main_event_task( void ) {

}


static void test_gpios(void) {
  /* Enable GPIOB clock */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
 /* Configure I2C1 pins: SCL and SDA ------------------------------------------*/
  GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_10 | GPIO_Pin_11;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(GPIOC, &GPIO_InitStructure);


  //GPIOB->BRR  = GPIO_Pin_6 | GPIO_Pin_7;

  GPIOC->BSRR  = GPIO_Pin_10 | GPIO_Pin_11;
}


static inline void main_i2c_init( void ) {

  /* Enable peripheral clocks --------------------------------------------------*/
  /* Enable I2C1 clock */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);
  /* Enable GPIOB clock */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);


  /* Configure I2C1 pins: SCL and SDA ------------------------------------------*/
  GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_6 | GPIO_Pin_7;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
  GPIO_Init(GPIOB, &GPIO_InitStructure);


  /* I2C configuration ----------------------------------------------------------*/
  I2C_InitTypeDef  I2C_InitStructure;
  I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
  I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
  I2C_InitStructure.I2C_OwnAddress1 = 0x02;
  I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
  //  I2C_InitStructure.I2C_Ack = I2C_Ack_Disable;
  I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
  I2C_InitStructure.I2C_ClockSpeed = 200000;

  /* I2C Peripheral Enable */
  I2C_Cmd(I2C1, ENABLE);
  /* Apply I2C configuration after enabling it */
  I2C_Init(I2C1, &I2C_InitStructure);

}

static inline void main_test_send( void ) {

  //  return;
  /* Send START condition */
  I2C_GenerateSTART(I2C1, ENABLE);

  /* Test on EV5 and clear it */
  while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT));

  //return;

  /* Send  address */
  //  I2C_Send7bitAddress(I2C1, 0x52, I2C_Direction_Transmitter);
  I2C_Send7bitAddress(I2C1, 0x58, I2C_Direction_Transmitter);

  //  return;

  /* Test on EV6 and clear it */
  while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));

  //  return;

  /* Snd data */
  I2C_SendData(I2C1, 0x06);

  /* Test on EV8 and clear it */
  while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));

  /* Send STOP condition */
  I2C_GenerateSTOP(I2C1, ENABLE);
}

