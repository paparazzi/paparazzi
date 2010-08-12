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

#include "peripherals/booz_hmc5843.h"
#include "my_debug_servo.h"
#include "math/pprz_algebra_int.h"

static inline void main_init( void );
static inline void main_periodic_task( void );
static inline void main_event_task( void );

static inline void main_init_hw(void);

static uint8_t i2c_done = FALSE;
#define INITIALISZED 6
static uint8_t mag_state = 0;
static volatile uint8_t mag_ready_for_read = FALSE;
static uint8_t reading_mag = FALSE;
extern void exti9_5_irq_handler(void);


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
    {
      DOWNLINK_SEND_I2C_ERRORS(DefaultChannel, 
			       &i2c2_errors.ack_fail_cnt,
			       &i2c2_errors.miss_start_stop_cnt,
			       &i2c2_errors.arb_lost_cnt,
			       &i2c2_errors.over_under_cnt,
			       &i2c2_errors.pec_recep_cnt,
			       &i2c2_errors.timeout_tlow_cnt,
			       &i2c2_errors.smbus_alert_cnt,
			       &i2c2_errors.unexpected_event_cnt,
			       &i2c2_errors.last_unexpected_event);
    });
  
  switch (mag_state) {
  case 2:
    i2c2.buf[0] = HMC5843_REG_CFGA;  // set to rate to 50Hz
    i2c2.buf[1] = 0x00 | (0x06 << 2);
    i2c2_transmit(HMC5843_ADDR, 2, &i2c_done);
    break;
  case 3:
    i2c2.buf[0] = HMC5843_REG_CFGB;  // set to gain to 1 Gauss
    i2c2.buf[1] = 0x01<<5;
    i2c2_transmit(HMC5843_ADDR, 2, &i2c_done);
    break;
  case 4:
    i2c2.buf[0] = HMC5843_REG_MODE;  // set to continuous mode
    i2c2.buf[1] = 0x00;
    i2c2_transmit(HMC5843_ADDR, 2, &i2c_done);
    break;
  case 5:
    break;
  case INITIALISZED:
    //    i2c2_receive(HMC5843_ADDR, 7, &i2c_done);
    //    reading_mag = TRUE;
    break;
  default:
    break;
  }

  if (mag_state  < INITIALISZED) mag_state++;

}


static inline void main_event_task( void ) {

  if (mag_state == INITIALISZED && mag_ready_for_read) {
    /* read mag */
    i2c2_receive(HMC5843_ADDR, 7, &i2c_done);
    reading_mag = TRUE;
    mag_ready_for_read = FALSE;
  }

  if (reading_mag && i2c_done) {
    RunOnceEvery(10, 
    {
      int16_t mx   = i2c2.buf[0]<<8 | i2c2.buf[1];
      int16_t my   = i2c2.buf[2]<<8 | i2c2.buf[3];
      int16_t mz   = i2c2.buf[4]<<8 | i2c2.buf[5];
      struct Int32Vect3 m;
      VECT3_ASSIGN(m, mx, my, mz);
      DOWNLINK_SEND_IMU_MAG_RAW(DefaultChannel, &m.x, &m.y, &m.z);
      //      uint8_t tmp[8];
      //      memcpy(tmp, i2c2.buf, 8);
      //      DOWNLINK_SEND_DEBUG(DefaultChannel, 8, tmp);
    }
		 );
    reading_mag = FALSE;
  }


}

static inline void main_init_hw( void ) {
  /* set mag ss as floating input (on PC12)    = shorted to I2C2 sda ----------*/
  /* set mag reset as floating input (on PC13) = shorted to I2C2 scl ----------*/
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
  GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12|GPIO_Pin_13;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOC, &GPIO_InitStructure);

  /* configure external interrupt exti5 on PB5( mag int ) */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource5);
  EXTI_InitTypeDef EXTI_InitStructure;
  EXTI_InitStructure.EXTI_Line = EXTI_Line5;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);

  NVIC_InitTypeDef NVIC_InitStructure;
  NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0F;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0F;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure); 

  DEBUG_SERVO1_INIT();
  DEBUG_SERVO2_INIT();

}




void exti9_5_irq_handler(void) {
  /* clear EXTI */
  if(EXTI_GetITStatus(EXTI_Line5) != RESET)
    EXTI_ClearITPendingBit(EXTI_Line5);

  mag_ready_for_read = TRUE;
}
