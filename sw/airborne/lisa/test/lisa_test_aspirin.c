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
#include "mcu.h"
#include "mcu_periph/sys_time.h"
#include "subsystems/datalink/downlink.h"
#include "led.h"

#include "peripherals/itg3200.h"
#include "peripherals/hmc5843.h"

static inline void main_init( void );
static inline void main_periodic_task( void );
static inline void main_event_task( void );

static inline void main_init_hw(void);

static volatile uint8_t gyro_ready_for_read;

extern void exti2_irq_handler(void);
extern void exti3_irq_handler(void);
extern void exti4_irq_handler(void);

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
  main_init_hw();

  gyro_ready_for_read = FALSE;

}

static uint8_t foo=0;


#define AccUnselect() GPIOB->BSRR = GPIO_Pin_12
#define AccSelect() GPIOB->BRR = GPIO_Pin_12
#define AccToggleSelect() GPIOB->ODR ^= GPIO_Pin_12


static inline void main_periodic_task( void ) {
  //  LED_TOGGLE(6);
  RunOnceEvery(10,
	       {
		 DOWNLINK_SEND_ALIVE(DefaultChannel, DefaultDevice, 16, MD5SUM);
		 LED_PERIODIC();
	       });


  switch (foo) {
  case 2:
    /* set gyro range to 2000deg/s and low pass at 256Hz */
    i2c2.buf[0] = ITG3200_REG_DLPF_FS;
    i2c2.buf[1] = 0x03;
    i2c2_transmit(ITG3200_ADDR, 2, &i2c_done);
    break;
  case 3:
    /* switch to gyroX clock */
    i2c2.buf[0] = ITG3200_REG_PWR_MGM;
    i2c2.buf[1] = 0x01;
    i2c2_transmit(ITG3200_ADDR, 2, &i2c_done);
    break;
  case 4:
    /* enable interrupt on data ready, idle hight */
    i2c2.buf[0] = ITG3200_REG_INT_CFG;
    i2c2.buf[1] = (0x01 | 0x01<<7);
    i2c2_transmit(ITG3200_ADDR, 2, &i2c_done);
    break;
  case 5:
    /* set mag rate to 50Hz */
    i2c2.buf[0] = HMC5843_REG_CFGA;
    i2c2.buf[1] = 0x00 | (0x06 << 2);
    i2c2_transmit(HMC5843_ADDR, 2, &i2c_done);
    break;
  case 6:
    /* Set mag gain to 1 Gauss */
    i2c2.buf[0] = HMC5843_REG_CFGB;
    i2c2.buf[1] = 0x01<<5;
    i2c2_transmit(HMC5843_ADDR, 2, &i2c_done);
    break;
  case 7:
    /* set mag to continuous measurements */
    i2c2.buf[0] = HMC5843_REG_MODE;
    i2c2.buf[1] = 0x00;
    i2c2_transmit(HMC5843_ADDR, 2, &i2c_done);
    break;
  case 8:
    /* reads 8 bytes from address 0x1b */
    //    i2c2.buf[0] = ITG3200_REG_TEMP_OUT_H;
    //    i2c2_transceive(ITG3200_ADDR,1, 8, &i2c_done);
    break;
  default:
    break;
  }

  if (foo< 8) foo++;


  //  AccToggleSelect();

}


static inline void main_event_task( void ) {


}

static inline void main_init_hw( void ) {

  /* set mag ss as output and assert it (on PC12)    = sda         ------------------------------*/
  /*     mag drdy  (on PB5)                          = mag int                                   */
  /* set mag reset as output and assert it (on PC13) = scl         ------------------------------*/
  /* set eeprom ss as output and assert it (on PC14) = gyro int    ------------------------------*/
  GPIOC->BSRR = GPIO_Pin_12;
  GPIOC->BSRR = GPIO_Pin_13;
  GPIOC->BSRR = GPIO_Pin_14;

  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
  GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13|GPIO_Pin_14;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOC, &GPIO_InitStructure);

  /* configure acc slave select */
  /* set acc slave select as output and assert it ( on PB12) */
  AccUnselect();
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  /* configure external interrupt exti4 on PD2( accel int ) */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD | RCC_APB2Periph_AFIO, ENABLE);
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOD, &GPIO_InitStructure);
  EXTI_InitTypeDef EXTI_InitStructure;
  GPIO_EXTILineConfig(GPIO_PortSourceGPIOD, GPIO_PinSource2);
  EXTI_InitStructure.EXTI_Line = EXTI_Line4;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);
  NVIC_InitTypeDef NVIC_InitStructure;
  NVIC_InitStructure.NVIC_IRQChannel = EXTI4_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0F;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0F;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;



  /* configure external interrupt exti2 on PC14( gyro int ) */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC | RCC_APB2Periph_AFIO, ENABLE);
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOC, &GPIO_InitStructure);

  GPIO_EXTILineConfig(GPIO_PortSourceGPIOC, GPIO_PinSource14);
  EXTI_InitStructure.EXTI_Line = EXTI_Line2;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);

  NVIC_InitStructure.NVIC_IRQChannel = EXTI2_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0F;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0F;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;

  NVIC_Init(&NVIC_InitStructure);



   /* configure external interrupt exti3 on PB5( mag int ) */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource5);
  EXTI_InitStructure.EXTI_Line = EXTI_Line3;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);

  NVIC_InitStructure.NVIC_IRQChannel = EXTI3_IRQn;
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
  SPI_InitStructure.SPI_DataSize = SPI_DataSize_16b;
  SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;
  SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;
  SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
  SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_16;
  SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
  SPI_InitStructure.SPI_CRCPolynomial = 7;
  SPI_Init(SPI2, &SPI_InitStructure);


}

void exti2_irq_handler(void) {
  /* clear EXTI */
  if(EXTI_GetITStatus(EXTI_Line2) != RESET)
    EXTI_ClearITPendingBit(EXTI_Line2);

  AccToggleSelect();
#if 0
  gyro_ready_for_read = TRUE;

  if (gyro_ready_for_read && i2c_done && foo>=5) {
    /* read gyros */
    /* reads 8 bytes from address 0x1b */
    i2c2.buf[0] = ITG3200_REG_TEMP_OUT_H;
    i2c2_transceive(ITG3200_ADDR,1, 8, &i2c_done);
    gyro_ready_for_read = FALSE;
  }
#endif
}

void exti3_irq_handler(void) {
  /* clear EXTI */
  if(EXTI_GetITStatus(EXTI_Line3) != RESET)
    EXTI_ClearITPendingBit(EXTI_Line3);

  //  AccToggleSelect();


}


void exti4_irq_handler(void) {
  /* clear EXTI */
  if(EXTI_GetITStatus(EXTI_Line4) != RESET)
    EXTI_ClearITPendingBit(EXTI_Line4);

  //  AccToggleSelect();


}

