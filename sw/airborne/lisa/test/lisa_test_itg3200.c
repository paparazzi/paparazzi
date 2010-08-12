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
#include "math/pprz_algebra_int.h"

#include "peripherals/booz_itg3200.h"
#include "my_debug_servo.h"

static inline void main_init( void );
static inline void main_periodic_task( void );
static inline void main_event_task( void );

static inline void main_init_hw(void);

static uint8_t i2c_done = FALSE;
#define INITIALISZED 6
static uint8_t gyro_state = 0;
static volatile uint8_t gyro_ready_for_read = FALSE;
static uint8_t reading_gyro = FALSE;

void exti15_10_irq_handler(void);

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
  RunOnceEvery(256, {
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

  switch (gyro_state) {
  case 2:
    /* set gyro range to 2000deg/s and low pass at 256Hz */
    i2c2.buf[0] = ITG3200_REG_DLPF_FS;
    i2c2.buf[1] = (0x03<<3);
    i2c2_transmit(ITG3200_ADDR, 2, &i2c_done);
    break;
  case 3:
    /* set sample rate to 533Hz */
    i2c2.buf[0] = ITG3200_REG_SMPLRT_DIV;
    i2c2.buf[1] = 0x0E;
    i2c2_transmit(ITG3200_ADDR, 2, &i2c_done);
    break;
  case 4:
    /* switch to gyroX clock */
    i2c2.buf[0] = ITG3200_REG_PWR_MGM;
    i2c2.buf[1] = 0x01;
    i2c2_transmit(ITG3200_ADDR, 2, &i2c_done);
    break;
  case 5:
    /* enable interrupt on data ready, idle hight */
    i2c2.buf[0] = ITG3200_REG_INT_CFG;
    i2c2.buf[1] = (0x01 | 0x01<<7);
    i2c2_transmit(ITG3200_ADDR, 2, &i2c_done);
    break;
  case INITIALISZED:
    /* reads 8 bytes from address 0x1b */
    //    i2c2.buf[0] = ITG3200_REG_TEMP_OUT_H;
    //    i2c2_transceive(ITG3200_ADDR,1, 8, &i2c_done);
    //    reading_gyro = TRUE;
  default:
    break;
  }

  if (gyro_state  < INITIALISZED) gyro_state++;

}


#if 0

#endif

static inline void main_event_task( void ) {

  if (gyro_state == INITIALISZED && gyro_ready_for_read) {
    /* reads 8 bytes from address 0x1b */
    i2c2.buf[0] = ITG3200_REG_TEMP_OUT_H;
    i2c2_transceive(ITG3200_ADDR,1, 8, &i2c_done);
    //   i2c2.buf[0] = ITG3200_REG_GYRO_XOUT_H;
    //    i2c2_transceive(ITG3200_ADDR,1, 6, &i2c_done);
    gyro_ready_for_read = FALSE;
    reading_gyro = TRUE;
  }

  if (reading_gyro && i2c_done) {
    //    DEBUG_S5_ON();
    reading_gyro = FALSE;
    int16_t tgp, tgq, tgr;

    int16_t ttemp = i2c2.buf[0]<<8 | i2c2.buf[1];
#if 1
    tgp = i2c2.buf[2]<<8 | i2c2.buf[3];
    tgq = i2c2.buf[4]<<8 | i2c2.buf[5];
    tgr = i2c2.buf[6]<<8 | i2c2.buf[7];
#endif
#if 0
    tgp = __REVSH(*(int16_t*)(i2c2.buf+2));
    tgq = __REVSH(*(int16_t*)(i2c2.buf+4));
    tgr = __REVSH(*(int16_t*)(i2c2.buf+6));
#endif
#if 0
    MyByteSwap16(*(int16_t*)(i2c2.buf+2), tgp);
    MyByteSwap16(*(int16_t*)(i2c2.buf+4), tgq);
    MyByteSwap16(*(int16_t*)(i2c2.buf+6), tgr);
#endif
    struct Int32Rates g;
    RATES_ASSIGN(g, tgp, tgq, tgr);
    RunOnceEvery(10, 
    {
      DOWNLINK_SEND_IMU_GYRO_RAW(DefaultChannel, &g.p, &g.q, &g.r);
      
      uint8_t tmp[8];
      memcpy(tmp, i2c2.buf, 8);
      DOWNLINK_SEND_DEBUG(DefaultChannel, 8, tmp);


    });
    //    DEBUG_S5_OFF();
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

  /* configure external interrupt exti15_10 on PC14( gyro int ) */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC | RCC_APB2Periph_AFIO, ENABLE);
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOC, &GPIO_InitStructure);

  GPIO_EXTILineConfig(GPIO_PortSourceGPIOC, GPIO_PinSource14);
  EXTI_InitTypeDef EXTI_InitStructure;
  EXTI_InitStructure.EXTI_Line = EXTI_Line14;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);

  NVIC_InitTypeDef NVIC_InitStructure;
  NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0F;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0F;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure); 

  DEBUG_SERVO1_INIT();
  DEBUG_SERVO2_INIT();

}


void exti15_10_irq_handler(void) {

  //  DEBUG_S4_ON();

  /* clear EXTI */
  if(EXTI_GetITStatus(EXTI_Line14) != RESET)
    EXTI_ClearITPendingBit(EXTI_Line14);

  //  DEBUG_S4_TOGGLE();

  gyro_ready_for_read = TRUE;

  //  DEBUG_S4_OFF();

}
