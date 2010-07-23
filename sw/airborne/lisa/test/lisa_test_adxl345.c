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

#include <stm32/gpio.h>
#include <stm32/flash.h>
#include <stm32/misc.h>
#include <stm32/exti.h>
#include <stm32/spi.h>

#include BOARD_CONFIG
#include "init_hw.h"
#include "sys_time.h"
#include "downlink.h"

#include "peripherals/booz_itg3200.h"
#include "peripherals/booz_hmc5843.h"

static inline void main_init( void );
static inline void main_periodic_task( void );
static inline void main_event_task( void );

static inline void main_init_hw(void);

extern void exti4_irq_handler(void);

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
  hw_init();
  sys_time_init();
  main_init_hw();

}

static void write_to_reg(uint8_t addr, uint8_t val);
static uint8_t read_fom_reg(uint8_t addr);

static uint8_t foo=0;
static volatile uint8_t inted = FALSE;
static uint8_t values[6];


#define ADXL345_REG_BW_RATE     0x2C
#define ADXL345_REG_POWER_CTL   0x2D
#define ADXL345_REG_INT_ENABLE  0x2E
#define ADXL345_REG_DATA_FORMAT 0x31
#define ADXL345_REG_DATA_X0     0x32
#define ADXL345_REG_DATA_X1     0x33
#define ADXL345_REG_DATA_Y0     0x34
#define ADXL345_REG_DATA_Y1     0x35
#define ADXL345_REG_DATA_Z0     0x36
#define ADXL345_REG_DATA_Z1     0x37



#define AccUnselect() GPIOB->BSRR = GPIO_Pin_12
#define AccSelect() GPIOB->BRR = GPIO_Pin_12
#define AccToggleSelect() GPIOB->ODR ^= GPIO_Pin_12

static void write_to_reg(uint8_t addr, uint8_t val) {

  AccSelect();
  SPI_I2S_SendData(SPI2, addr);
  while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET);
  SPI_I2S_SendData(SPI2, val);
  while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET);
  while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_BSY) == SET);
  AccUnselect();

}

static uint8_t read_fom_reg(uint8_t addr) {
  AccSelect();
  SPI_I2S_SendData(SPI2, (1<<7|addr));
  while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET);
  SPI_I2S_SendData(SPI2, 0x00);
  while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET);
  while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_BSY) == SET);
  uint8_t ret = SPI_I2S_ReceiveData(SPI2);
  AccUnselect();
  return ret;
}

static void read_data(void) {
  AccSelect();
  SPI_I2S_SendData(SPI2, (1<<7|1<<6|ADXL345_REG_DATA_X0));
  while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET);
  while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_BSY) == SET);
  uint8_t foo = SPI_I2S_ReceiveData(SPI2);

  SPI_I2S_SendData(SPI2, 0x00);
  while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET);
  while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_BSY) == SET);
  values[0] = SPI_I2S_ReceiveData(SPI2);
  SPI_I2S_SendData(SPI2, 0x00);
  while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET);
  while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_BSY) == SET); 
  values[1] = SPI_I2S_ReceiveData(SPI2);

  SPI_I2S_SendData(SPI2, 0x00);
  while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET);
  while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_BSY) == SET);
  values[2] = SPI_I2S_ReceiveData(SPI2);
  SPI_I2S_SendData(SPI2, 0x00);
  while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET);
  while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_BSY) == SET); 
  values[3] = SPI_I2S_ReceiveData(SPI2);

  SPI_I2S_SendData(SPI2, 0x00);
  while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET);
  while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_BSY) == SET);
  values[4] = SPI_I2S_ReceiveData(SPI2);
  SPI_I2S_SendData(SPI2, 0x00);
  while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET);
  while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_BSY) == SET); 
  values[5] = SPI_I2S_ReceiveData(SPI2);

  AccUnselect();

}


static inline void main_periodic_task( void ) {


  RunOnceEvery(10, 
	       {
		 DOWNLINK_SEND_ALIVE(DefaultChannel, 16, MD5SUM);
		 LED_PERIODIC();
	       });

  switch (foo) {
  case 1: 
    {
      /* read data rate */
      //      uint8_t bar = read_fom_reg(ADXL345_REG_BW_RATE);
      write_to_reg(ADXL345_REG_BW_RATE, 0x0D);
    }
    break;
  case 2:
    /* switch to measurememnt mode */
    write_to_reg(ADXL345_REG_POWER_CTL, 1<<3);
    break;
  case 3:
    /* switch to measurememnt mode */
    write_to_reg(ADXL345_REG_INT_ENABLE, 1<<7);
    break;
  case 4:
     write_to_reg(ADXL345_REG_DATA_FORMAT, 1<<3|1<<5);
    break;
  case 5:
    //    read_data();
    break;
  }
  
  if (foo < 5) foo++;

}


static inline void main_event_task( void ) {

  if (inted) {
    LED_TOGGLE(6);
    read_data();
    inted = FALSE;
        int16_t* iax = &values[0];
        int16_t* iay = &values[2];
        int16_t* iaz = &values[4];
        RunOnceEvery(10, {DOWNLINK_SEND_IMU_MAG_RAW(DefaultChannel, iax, iay, iaz);});
    //uint16_t* ax = &values[0];
    //uint16_t* ay = &values[2];
    //uint16_t* az = &values[4];
    //RunOnceEvery(10, {DOWNLINK_SEND_IMU_ACCEL_RAW(DefaultChannel, ax, ay, az);});
  }

}

static inline void main_init_hw( void ) {
  
  /* configure acc slave select */
  /* set acc slave select as output and assert it ( on PB12) */
  AccUnselect();
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
  GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  /* configure external interrupt exti2 on PD2( accel int ) */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD | RCC_APB2Periph_AFIO, ENABLE);
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOD, &GPIO_InitStructure);
  EXTI_InitTypeDef EXTI_InitStructure;
  GPIO_EXTILineConfig(GPIO_PortSourceGPIOD, GPIO_PinSource2);
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


  /* Enable SPI2 Periph clock -------------------------------------------------*/
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);
  
  /* Configure GPIOs: SCK, MISO and MOSI  --------------------------------*/
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO , ENABLE);
  SPI_Cmd(SPI2, ENABLE);

  /* configure SPI */
  SPI_InitTypeDef SPI_InitStructure;					
  SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;	
  SPI_InitStructure.SPI_Mode = SPI_Mode_Master;			
  SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;			
  SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;				
  SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;			
  SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;				
  SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_32;	
  SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;			
  SPI_InitStructure.SPI_CRCPolynomial = 7;				
  SPI_Init(SPI2, &SPI_InitStructure);					


}


void exti2_irq_handler(void) {

  /* clear EXTI */
  if(EXTI_GetITStatus(EXTI_Line2) != RESET)
    EXTI_ClearITPendingBit(EXTI_Line2);

  //  AccToggleSelect();
  LED_ON(6);
  inted = TRUE;


}

