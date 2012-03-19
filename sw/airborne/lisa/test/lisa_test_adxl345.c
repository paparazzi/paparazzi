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

/*              lisa/L  lisa/M
 *  ACC_DRDY     PD2     PB2
 *  ACC_SS       PB12    PB12
 *
 */


#include <libopencm3/stm32/f1/gpio.h>
#include <libopencm3/stm32/exti.h>
#include <libopencm3/stm32/spi.h>

#include BOARD_CONFIG
#include "mcu.h"
#include "mcu_periph/sys_time.h"
#include "subsystems/datalink/downlink.h"

#include "peripherals/adxl345.h"
#include "my_debug_servo.h"
#include "led.h"

static inline void main_init( void );
static inline void main_periodic_task( void );
static inline void main_event_task( void );

static inline void main_init_hw(void);

void exti2_isr(void);

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

}

static void write_to_reg(uint8_t addr, uint8_t val);
static uint8_t read_fom_reg(uint8_t addr);
#define CONFIGURED 6
static uint8_t acc_status=0;
static volatile uint8_t acc_ready_for_read = FALSE;
static uint8_t values[6];

#define AccUnselect() GPIOB->BSRR = GPIO_Pin_12
#define AccSelect() GPIOB->BRR = GPIO_Pin_12
#define AccToggleSelect() GPIOB->ODR ^= GPIO_Pin_12

static void write_to_reg(uint8_t addr, uint8_t val) {

#warning "Needs porting to libopencm3 or use the real driver!"

#if 0
  AccSelect();
  SPI_I2S_SendData(SPI2, addr);
  while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET);
  SPI_I2S_SendData(SPI2, val);
  while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET);
  while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_BSY) == SET);
  AccUnselect();
#endif

}

static uint8_t read_fom_reg(uint8_t addr) {

#warning "Needs porting to libopencm3 or use the real driver!"

#if 0
  AccSelect();
  SPI_I2S_SendData(SPI2, (1<<7|addr));
  while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET);
  SPI_I2S_SendData(SPI2, 0x00);
  while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET);
  while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_BSY) == SET);
  uint8_t ret = SPI_I2S_ReceiveData(SPI2);
  AccUnselect();
#endif
  return ret;
}

static void read_data(void) {
#warning "Needs porting to libopencm3 or use the real driver!"

#if 0
  AccSelect();
  SPI_I2S_SendData(SPI2, (1<<7|1<<6|ADXL345_REG_DATA_X0));
  while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET);
  while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_BSY) == SET);
  uint8_t __attribute__ ((unused)) foo = SPI_I2S_ReceiveData(SPI2);

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

#endif

}


static inline void main_periodic_task( void ) {


  RunOnceEvery(10,
    {
      DOWNLINK_SEND_ALIVE(DefaultChannel, DefaultDevice, 16, MD5SUM);
      LED_PERIODIC();
    });

  switch (acc_status) {
  case 1:
    {
      /* read data rate */
      //      uint8_t bar = read_fom_reg(ADXL345_REG_BW_RATE);
    }
    /* set data rate to 800Hz */
    write_to_reg(ADXL345_REG_BW_RATE, 0x0D);
    break;
  case 2:
    /* switch to measurememnt mode */
    write_to_reg(ADXL345_REG_POWER_CTL, 1<<3);
    break;
  case 3:
    /* enable data ready interrupt */
    write_to_reg(ADXL345_REG_INT_ENABLE, 1<<7);
    break;
  case 4:
    /* Enable full res and interrupt active low */
     write_to_reg(ADXL345_REG_DATA_FORMAT, 1<<3|1<<5);
    break;
  case 5:
    /* reads data once to bring interrupt line up */
    read_data();
    break;
  case CONFIGURED:
    //    read_data();
    break;
  default:
    break;
  }

  if (acc_status < CONFIGURED) acc_status++;

}


static inline void main_event_task( void ) {

  if (acc_status >= CONFIGURED && acc_ready_for_read) {
    read_data();
    acc_ready_for_read = FALSE;
    int32_t iax = *((int16_t*)&values[0]);
    int32_t iay = *((int16_t*)&values[2]);
    int32_t iaz = *((int16_t*)&values[4]);
    RunOnceEvery(10, {DOWNLINK_SEND_IMU_ACCEL_RAW(DefaultChannel, DefaultDevice, &iax, &iay, &iaz);});
  }

}

static inline void main_init_hw( void ) {

#warning "Needs porting to libopencm3 or use the real driver!"

#if 0
  /* configure acc slave select */
  /* set acc slave select as output and assert it ( on PB12) */
  AccUnselect();
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
  GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  /* configure external interrupt exti2 on P(B/D)2( accel int ) */
  RCC_APB2PeriphClockCmd(IMU_ACC_DRDY_RCC_GPIO | RCC_APB2Periph_AFIO, ENABLE);
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(IMU_ACC_DRDY_GPIO, &GPIO_InitStructure);
  EXTI_InitTypeDef EXTI_InitStructure;
  GPIO_EXTILineConfig(IMU_ACC_DRDY_GPIO_PORTSOURCE, GPIO_PinSource2);
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

  DEBUG_SERVO2_INIT();
#endif

}


void exti2_irq_handler(void) {

  /* clear EXTI */
  exti_reset_request(EXTI2);

  //DEBUG_S4_TOGGLE();

  acc_ready_for_read = TRUE;


}

