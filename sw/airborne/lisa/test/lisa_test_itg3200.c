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
#include "std.h"

#include "peripherals/booz_itg3200.h"
#include "my_debug_servo.h"

static inline void main_init( void );
static inline void main_periodic_task( void );
static inline void main_event_task( void );

static inline void main_init_hw(void);

static uint8_t i2c_done = FALSE;
#define INITIALISZED 5
static uint8_t gyro_state = 0;
static volatile uint8_t gyro_ready_for_read = FALSE;
static uint8_t reading_gyro = FALSE;
extern void exti2_irq_handler(void);


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

static inline void main_periodic_task( void ) {
  //  LED_TOGGLE(6);
  RunOnceEvery(10, 
  {
    DOWNLINK_SEND_ALIVE(DefaultChannel, 16, MD5SUM);
    LED_PERIODIC();
  });
  RunOnceEvery(256, 
  {DOWNLINK_SEND_I2C_ERRORS(DefaultChannel, &i2c2.errc_ack_fail, &i2c2.errc_miss_start_stop,
			    &i2c2.errc_arb_lost, &i2c2.errc_over_under,
			    &i2c2.errc_pec_recep, &i2c2.errc_timeout_tlow,
			    &i2c2.errc_smbus_alert);
  });

  switch (gyro_state) {
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
  case INITIALISZED:
    /* reads 8 bytes from address 0x1b */
    i2c2.buf[0] = ITG3200_REG_TEMP_OUT_H;
    i2c2_transceive(ITG3200_ADDR,1, 8, &i2c_done);
    reading_gyro = TRUE;
  default:
    break;
  }

  if (gyro_state  < INITIALISZED) gyro_state++;

}


static inline void main_event_task( void ) {

  if (gyro_state == INITIALISZED && gyro_ready_for_read) {
    /* read gyros */
    /* reads 8 bytes from address 0x1b */
    //    i2c2.buf[0] = ITG3200_REG_TEMP_OUT_H;
    //    i2c2_transceive(ITG3200_ADDR,1, 8, &i2c_done);
    gyro_ready_for_read = FALSE;
  }

  if (reading_gyro && i2c_done) {
    RunOnceEvery(10, 
    {
      int16_t temp = i2c2.buf[0]<<8 | i2c2.buf[1];
      int16_t gp   = i2c2.buf[2]<<8 | i2c2.buf[3];
      int16_t gq   = i2c2.buf[4]<<8 | i2c2.buf[5];
      int16_t gr   = i2c2.buf[6]<<8 | i2c2.buf[7];
      DOWNLINK_SEND_IMU_MAG_RAW(DefaultChannel, &gp, &gq, &gr);
      //      uint8_t tmp[8];
      //      memcpy(tmp, i2c2.buf, 8);
      //      DOWNLINK_SEND_DEBUG(DefaultChannel, 8, tmp);
    }
		 );
    reading_gyro = FALSE;
  }


}

static inline void main_init_hw( void ) {
  /* set mag ss as floating input (on PC12) = shorted to sda         ------------------------------*/
  /* set mag reset as floating input (on PC13) = shorted to scl      ------------------------------*/
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
  GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12|GPIO_Pin_13;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOC, &GPIO_InitStructure);


  /* set "eeprom ss" as floating input (on PC14) = gyro int          ------------------------------*/
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
  GPIO_Init(GPIOC, &GPIO_InitStructure);

  /* configure external interrupt exti2 on PC14( gyro int ) */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC | RCC_APB2Periph_AFIO, ENABLE);
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOC, &GPIO_InitStructure);

  GPIO_EXTILineConfig(GPIO_PortSourceGPIOC, GPIO_PinSource14);
  EXTI_InitTypeDef EXTI_InitStructure;
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
  //  NVIC_Init(&NVIC_InitStructure); 

  DEBUG_SERVO2_INIT();

}


void exti2_irq_handler(void) {

  /* clear EXTI */
  if(EXTI_GetITStatus(EXTI_Line2) != RESET)
    EXTI_ClearITPendingBit(EXTI_Line2);

  DEBUG_S4_TOGGLE();

  gyro_ready_for_read = TRUE;

}
