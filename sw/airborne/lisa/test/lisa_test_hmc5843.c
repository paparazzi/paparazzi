/*
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

#include <libopencm3/stm32/f1/gpio.h>
#include <libopencm3/stm32/exti.h>
#include <libopencm3/stm32/spi.h>

/*
 *           lisa/L   lisa/M
 * mag drdy  PB5      PB5
 *
 */

#include BOARD_CONFIG
#include "mcu.h"
#include "mcu_periph/sys_time.h"
#include "subsystems/datalink/downlink.h"
#include "std.h"
#include "led.h"

#include "mcu_periph/uart.h"
#include "mcu_periph/i2c.h"
#include "peripherals/hmc5843.h"
#include "my_debug_servo.h"
#include "math/pprz_algebra_int.h"

static inline void main_init( void );
static inline void main_periodic_task( void );
static inline void main_event_task( void );

static inline void main_init_hw(void);
static void send_config(void);

static struct i2c_transaction i2c_trans;
struct i2c_transaction t1;
struct i2c_transaction t2;

#define INITIALIZED 6
static uint8_t mag_state = 0;
static volatile uint8_t mag_ready_for_read = FALSE;
static uint8_t reading_mag = FALSE;

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

static inline void main_periodic_task( void ) {
  //  LED_TOGGLE(6);
  RunOnceEvery(10,
  {
    DOWNLINK_SEND_ALIVE(DefaultChannel, DefaultDevice, 16, MD5SUM);
    LED_PERIODIC();
  });
  RunOnceEvery(256,
    {
      uint16_t i2c2_queue_full_cnt        = i2c2.errors->queue_full_cnt;
      uint16_t i2c2_ack_fail_cnt          = i2c2.errors->ack_fail_cnt;
      uint16_t i2c2_miss_start_stop_cnt   = i2c2.errors->miss_start_stop_cnt;
      uint16_t i2c2_arb_lost_cnt          = i2c2.errors->arb_lost_cnt;
      uint16_t i2c2_over_under_cnt        = i2c2.errors->over_under_cnt;
      uint16_t i2c2_pec_recep_cnt         = i2c2.errors->pec_recep_cnt;
      uint16_t i2c2_timeout_tlow_cnt      = i2c2.errors->timeout_tlow_cnt;
      uint16_t i2c2_smbus_alert_cnt       = i2c2.errors->smbus_alert_cnt;
      uint16_t i2c2_unexpected_event_cnt  = i2c2.errors->unexpected_event_cnt;
      uint32_t i2c2_last_unexpected_event = i2c2.errors->last_unexpected_event;
      const uint8_t _bus2 = 2;
      DOWNLINK_SEND_I2C_ERRORS(DefaultChannel, DefaultDevice,
                               &i2c2_queue_full_cnt,
                               &i2c2_ack_fail_cnt,
                               &i2c2_miss_start_stop_cnt,
                               &i2c2_arb_lost_cnt,
                               &i2c2_over_under_cnt,
                               &i2c2_pec_recep_cnt,
                               &i2c2_timeout_tlow_cnt,
                               &i2c2_smbus_alert_cnt,
                               &i2c2_unexpected_event_cnt,
                               &i2c2_last_unexpected_event,
                               &_bus2);
    });
  if (mag_state == 2) send_config();

#if 0
  switch (mag_state) {
  case 2:
    i2c_trans.type = I2CTransTx;
    i2c_trans.slave_addr = HMC5843_ADDR;
    i2c_trans.buf[0] = HMC5843_REG_CFGA;  // set to rate to 50Hz
    i2c_trans.buf[1] = 0x00 | (0x06 << 2);
    i2c_trans.len_w = 2;
    i2c_submit(&i2c2,&i2c_trans);
    break;
  case 3:
    i2c_trans.type = I2CTransTx;
    i2c_trans.slave_addr = HMC5843_ADDR;
    i2c_trans.buf[0] = HMC5843_REG_CFGB;  // set to gain to 1 Gauss
    i2c_trans.buf[1] = 0x01<<5;
    i2c_trans.len_w = 2;
    i2c_submit(&i2c2,&i2c_trans);
    break;
  case 4:
    i2c_trans.type = I2CTransTx;
    i2c_trans.slave_addr = HMC5843_ADDR;
    i2c_trans.buf[0] = HMC5843_REG_MODE;  // set to continuous mode
    i2c_trans.buf[1] = 0x00;
    i2c_trans.len_w = 2;
    i2c_submit(&i2c2,&i2c_trans);
    break;
  case 5:
    break;
  case INITIALIZED:
    //    i2c2_receive(HMC5843_ADDR, 7, &i2c_done);
    //    reading_mag = TRUE;
    break;
  default:
    break;
  }
#endif
  //  if (mag_state  == 4) mag_state=1;

  if (mag_state  < INITIALIZED) mag_state++;

}


static inline void main_event_task( void ) {

  if (mag_state == INITIALIZED && mag_ready_for_read && (i2c_trans.status==I2CTransSuccess || i2c_trans.status == I2CTransFailed)) {
    /* read mag */
    i2c_trans.type = I2CTransRx;
    i2c_trans.slave_addr = HMC5843_ADDR;
    i2c_trans.len_r = 7;
    i2c_submit(&i2c2,&i2c_trans);
    reading_mag = TRUE;
    mag_ready_for_read = FALSE;
  }

  if (reading_mag && i2c_trans.status==I2CTransSuccess) {
    RunOnceEvery(10,
    {
      int16_t mx   = i2c_trans.buf[0]<<8 | i2c_trans.buf[1];
      int16_t my   = i2c_trans.buf[2]<<8 | i2c_trans.buf[3];
      int16_t mz   = i2c_trans.buf[4]<<8 | i2c_trans.buf[5];
      struct Int32Vect3 m;
      VECT3_ASSIGN(m, mx, my, mz);
      DOWNLINK_SEND_IMU_MAG_RAW(DefaultChannel, DefaultDevice, &m.x, &m.y, &m.z);
      //      uint8_t tmp[8];
      //      memcpy(tmp, i2c2.buf, 8);
      //      DOWNLINK_SEND_DEBUG(DefaultChannel, DefaultDevice, 8, tmp);
    }
		 );
    reading_mag = FALSE;
  }

}



static void send_config(void) {

  t1.type = I2CTransTx;
  t1.slave_addr = HMC5843_ADDR;
  t1.buf[0] = HMC5843_REG_CFGA;  // set to rate to 50Hz
  t1.buf[1] = 0x00 | (0x06 << 2);
  t1.len_w = 2;
  i2c_submit(&i2c2,&t1);

  t2.type = I2CTransTx;
  t2.slave_addr = HMC5843_ADDR;
  t2.buf[0] = HMC5843_REG_CFGB;  // set to gain to 1 Gauss
  t2.buf[1] = 0x01<<5;
  t2.len_w = 2;
  i2c_submit(&i2c2,&t2);

  i2c_trans.type = I2CTransTx;
  i2c_trans.slave_addr = HMC5843_ADDR;
  i2c_trans.buf[0] = HMC5843_REG_MODE;  // set to continuous mode
  i2c_trans.buf[1] = 0x00;
  i2c_trans.len_w = 2;
  i2c_submit(&i2c2,&i2c_trans);

}








static inline void main_init_hw( void ) {

#warning "This has to be ported to libopencm3 or using the actual driver!"

#if 0
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
#endif

}




void exti9_5_isr(void) {
  /* clear EXTI */
  exti_reset_request(EXTI5);

  if (mag_state == INITIALIZED) mag_ready_for_read = TRUE;
}
