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

//
// scl2 PB10
// sda2 PB11
// baro1 drdy PC0 ( differential )
// baro2 drdy PD1 ( absolute )
// baro1 has addr to 3V3 0x92
// baro2 has addr to GND 0x90
//


// absolute
//#define BARO_ADDR 0x90   
// differential
#define BARO_ADDR 0x92

#include <stm32/flash.h>
#include <stm32/misc.h>
#include <stm32/i2c.h>

#include BOARD_CONFIG
#include "init_hw.h"
#include "sys_time.h"
#include "downlink.h"

static inline void main_init( void );
static inline void main_periodic_task( void );
static inline void main_event_task( void );

static inline void main_i2c_init( void );
static inline void main_send_config( void );
static inline void main_send_reset(void);
static inline void main_read_register(uint8_t reg);
static inline void write_to_register(uint8_t reg_addr, uint8_t val_msb, uint8_t val_lsb);


uint16_t foo;
uint16_t bar;
int initialised;


void Delay(__IO uint32_t nCount) {
  for(; nCount != 0; nCount--);
}



int main(void) {
  main_init();
  //  Delay(50000);
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
  main_i2c_init();
  initialised = FALSE;
}

static inline void main_periodic_task( void ) {
  RunOnceEvery(7, {
      if (!initialised) {
	main_send_reset();
	//main_send_config();
	uint8_t cfg_msb = 0x84;
	// double gain ??
	//	uint8_t cfg_msb = 0x86;
	uint8_t cfg_lsb = 0x83;
	write_to_register(0x01, cfg_msb, cfg_lsb);
	// low tresh  msb to 0
	//	write_to_register(0x02, 0x00, 0x00);
	// high tresh  msb to 1
	write_to_register(0x03, 0xFF, 0xFF);
	initialised = TRUE;
      }
      
      main_read_register(0X00);
      //      main_read_register(0x02);
      
      int16_t adc = foo<<8 | bar;
      uint16_t adc1 = 255* foo + bar;
      int16_t bla = 0;
      DOWNLINK_SEND_IMU_MAG_RAW(DefaultChannel, &adc, &foo, &bar);
      //DOWNLINK_SEND_BOOT(DefaultChannel, &adc1);
      //      uint16_t bla = 0;
      //      DOWNLINK_SEND_BOOZ2_BARO_RAW(DefaultChannel, &bla, &adc, &bla);
      LED_PERIODIC();
    });
   RunOnceEvery(100, {
      LED_TOGGLE(3);
      DOWNLINK_SEND_ALIVE(DefaultChannel, 16, MD5SUM);
    });

}

static inline void main_event_task( void ) {

}


static inline void main_i2c_init( void ) {

  //  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
  /* enable pullup on PC0 */
 GPIO_InitTypeDef GPIO_InitStructure;
 GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_0;
 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
 GPIO_Init(GPIOC, &GPIO_InitStructure);

  /* Enable peripheral clocks --------------------------------------------------*/
  /* Enable I2C2 clock */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2, ENABLE);
  /* Enable GPIOB clock */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);


  /* Configure I2C2 pins: SCL and SDA ------------------------------------------*/
  //  GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_10 | GPIO_Pin_11;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
  GPIO_Init(GPIOB, &GPIO_InitStructure);


  /* I2C configuration ----------------------------------------------------------*/
  I2C_InitTypeDef  I2C_InitStructure; 
  I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
  I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
  I2C_InitStructure.I2C_OwnAddress1 = 0x0;
  I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
  //  I2C_InitStructure.I2C_Ack = I2C_Ack_Disable;
  I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
  I2C_InitStructure.I2C_ClockSpeed = 200000;
  
  /* I2C Peripheral Enable */
  I2C_Cmd(I2C2, ENABLE);
  /* Apply I2C configuration after enabling it */
  I2C_Init(I2C2, &I2C_InitStructure);

}

static inline void main_send_config( void ) {

  /* Send START condition */
  I2C_GenerateSTART(I2C2, ENABLE);

  /* Test on EV5 and clear it */
  while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_MODE_SELECT));

  /* Send  address */
  I2C_Send7bitAddress(I2C2, BARO_ADDR, I2C_Direction_Transmitter);
  
  /* Test on EV6 and clear it */
  while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));

  /* Snd data */
  I2C_SendData(I2C2, 0x01); // points to Config register

  /* Test on EV8 and clear it */
  while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTED));

  // Third byte: 0b10000100 (MSB of the Config register to be written)
  uint8_t cfg_msb = 0x84;

  I2C_SendData(I2C2, cfg_msb);
  /* Test on EV8 and clear it */
  while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
  // Fourth byte: 0b10000011 (LSB of the Config register to be written)
  uint8_t cfg_lsb = 0x83;
  I2C_SendData(I2C2, cfg_lsb);
  while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTED));

  /* Send STOP condition */
  I2C_GenerateSTOP(I2C2, ENABLE);

}

static void write_to_register(uint8_t reg_addr, uint8_t val_msb, uint8_t val_lsb) {
    /* Send START condition */
  I2C_GenerateSTART(I2C2, ENABLE);
  while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_MODE_SELECT));

  /* Send  slave address */
  I2C_Send7bitAddress(I2C2, BARO_ADDR, I2C_Direction_Transmitter);
  while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));

  /* Send  register address */
  I2C_SendData(I2C2, reg_addr);
  while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
  /* Send  val msb */
  I2C_SendData(I2C2, val_msb);
  while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
  /* Send  val lsb */
  I2C_SendData(I2C2, val_lsb);
  while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTED));

  /* Send STOP condition */
  I2C_GenerateSTOP(I2C2, ENABLE);
}





static inline void main_send_reset(void) {
 
  /* Send START condition */
  I2C_GenerateSTART(I2C2, ENABLE);
  
  /* Test on EV5 and clear it */
  while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_MODE_SELECT));
  
  /* Send  address : general call */
  I2C_Send7bitAddress(I2C2, 0x00, I2C_Direction_Transmitter);
  
  /* Test on EV6 and clear it */
  while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));
  
  /* Snd data */
  I2C_SendData(I2C2, 0x06); // reset command

  /* Test on EV8 and clear it */
  while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTED));

  I2C_GenerateSTOP(I2C2, ENABLE);

}







static void main_read_register( uint8_t reg) {

  I2C_AcknowledgeConfig(I2C2, ENABLE);

  /* Send START condition */
  I2C_GenerateSTART(I2C2, ENABLE);
  /* Test on EV5 and clear it */
  while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_MODE_SELECT));

  /* Send  address */
  I2C_Send7bitAddress(I2C2, BARO_ADDR, I2C_Direction_Transmitter);
  /* Test on EV6 and clear it */
  while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));

  /* Snd data */
  I2C_SendData(I2C2, reg); // points to conversion register
  /* Test on EV8 and clear it */
  while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTED));

  /* Send STOP condition */
  I2C_GenerateSTOP(I2C2, ENABLE);


 /* Send START condition */
  I2C_GenerateSTART(I2C2, ENABLE);
  /* Test on EV5 and clear it */
  while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_MODE_SELECT));
  
  /* Send  address */
  I2C_Send7bitAddress(I2C2, BARO_ADDR, I2C_Direction_Receiver);
  /* Test on EV6 and clear it */
  while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));

  while (!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_RECEIVED));
  foo = I2C_ReceiveData(I2C2);

  I2C_AcknowledgeConfig(I2C2, DISABLE);
  I2C_GenerateSTOP(I2C2, ENABLE);

  while (!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_RECEIVED));
  bar = I2C_ReceiveData(I2C2);
  
}

