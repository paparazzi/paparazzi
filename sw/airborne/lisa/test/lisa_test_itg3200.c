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

/*               lisa/L   lisa/M
 *   gyro-drdy   PC14
 *
 *
 */


#include <libopencm3/stm32/f1/gpio.h>
#include <libopencm3/stm32/exti.h>
#include <libopencm3/stm32/spi.h>

#include BOARD_CONFIG
#include "mcu.h"
#include "mcu_periph/uart.h"
#include "mcu_periph/i2c.h"
#include "mcu_periph/sys_time.h"
#include "subsystems/datalink/downlink.h"
#include "std.h"
#include "math/pprz_algebra_int.h"

#include "peripherals/itg3200_regs.h"
#include "my_debug_servo.h"
#include "led.h"

static inline void main_init( void );
static inline void main_periodic_task( void );
static inline void main_event_task( void );

static inline void main_init_hw(void);

static struct i2c_transaction i2c_trans;
#define INITIALIZED 6
static uint8_t gyro_state = 0;
static volatile uint8_t gyro_ready_for_read = FALSE;
static uint8_t reading_gyro = FALSE;

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
  RunOnceEvery(256, {
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

  switch (gyro_state) {

  case 1:
    /* dummy one byte write for testing */
    i2c_trans.type = I2CTransTx;
    i2c_trans.slave_addr = ITG3200_ADDR;
    i2c_trans.buf[0] = ITG3200_REG_TEMP_OUT_H;
    i2c_trans.len_w = 1;
    i2c_submit(&i2c2,&i2c_trans);
    break;
  case 2:
    /* set gyro range to 2000deg/s and low pass at 256Hz */
    i2c_trans.type = I2CTransTx;
    i2c_trans.slave_addr = ITG3200_ADDR;
    i2c_trans.buf[0] = ITG3200_REG_DLPF_FS;
    i2c_trans.buf[1] = (0x03<<3);
    i2c_trans.len_w = 2;
    i2c_submit(&i2c2,&i2c_trans);
    break;
  case 3:
    /* set sample rate to 533Hz */
    i2c_trans.type = I2CTransTx;
    i2c_trans.slave_addr = ITG3200_ADDR;
    i2c_trans.buf[0] = ITG3200_REG_SMPLRT_DIV;
    i2c_trans.buf[1] = 0x0E;
    i2c_trans.len_w = 2;
    i2c_submit(&i2c2,&i2c_trans);
    break;
  case 4:
    /* switch to gyroX clock */
    i2c_trans.type = I2CTransTx;
    i2c_trans.slave_addr = ITG3200_ADDR;
    i2c_trans.buf[0] = ITG3200_REG_PWR_MGM;
    i2c_trans.buf[1] = 0x01;
    i2c_trans.len_w = 2;
    i2c_submit(&i2c2,&i2c_trans);
    break;
  case 5:
    /* enable interrupt on data ready, idle hight */
    i2c_trans.type = I2CTransTx;
    i2c_trans.slave_addr = ITG3200_ADDR;
    i2c_trans.buf[0] = ITG3200_REG_INT_CFG;
    i2c_trans.buf[1] = (0x01 | 0x01<<7);
    i2c_trans.len_w = 2;
    i2c_submit(&i2c2,&i2c_trans);
    break;
  case INITIALIZED:
    /* reads 8 bytes from address 0x1b */
    //    i2c2.buf[0] = ITG3200_REG_TEMP_OUT_H;
    //    i2c2_transceive(ITG3200_ADDR,1, 8, &i2c_done);
    //    reading_gyro = TRUE;
  default:
    break;
  }

  //  if (gyro_state == 1) gyro_state = 0;
  if (gyro_state  < INITIALIZED) gyro_state++;

}


#if 0

#endif

static inline void main_event_task( void ) {

  if (gyro_state == INITIALIZED && gyro_ready_for_read &&
      ( i2c_trans.status==I2CTransSuccess || i2c_trans.status==I2CTransFailed)) {
    /* reads 8 bytes from address 0x1b */
    i2c_trans.type = I2CTransTxRx;
    i2c_trans.buf[0] = ITG3200_REG_TEMP_OUT_H;
    i2c_trans.len_w = 1;
    i2c_trans.len_r = 8;
    i2c_submit(&i2c2,&i2c_trans);
    //   i2c2.buf[0] = ITG3200_REG_GYRO_XOUT_H;
    //    i2c2_transceive(ITG3200_ADDR,1, 6, &i2c_done);
    gyro_ready_for_read = FALSE;
    reading_gyro = TRUE;
  }

  if (reading_gyro &&
      (i2c_trans.status==I2CTransSuccess || i2c_trans.status==I2CTransFailed)) {
    //    DEBUG_S5_ON();
    reading_gyro = FALSE;
    int16_t tgp, tgq, tgr;

    int16_t ttemp = i2c_trans.buf[0]<<8 | i2c_trans.buf[1];
#if 1
    tgp = i2c_trans.buf[2]<<8 | i2c_trans.buf[3];
    tgq = i2c_trans.buf[4]<<8 | i2c_trans.buf[5];
    tgr = i2c_trans.buf[6]<<8 | i2c_trans.buf[7];
#endif
#if 0
    tgp = __REVSH(*(int16_t*)(i2c_trans.buf+2));
    tgq = __REVSH(*(int16_t*)(i2c_trans.buf+4));
    tgr = __REVSH(*(int16_t*)(i2c_trans.buf+6));
#endif
#if 0
    MyByteSwap16(*(int16_t*)(i2c_trans.buf+2), tgp);
    MyByteSwap16(*(int16_t*)(i2c_trans.buf+4), tgq);
    MyByteSwap16(*(int16_t*)(i2c_trans.buf+6), tgr);
#endif
    struct Int32Rates g;
    RATES_ASSIGN(g, tgp, tgq, tgr);
    RunOnceEvery(10,
    {
      DOWNLINK_SEND_IMU_GYRO_RAW(DefaultChannel, DefaultDevice, &g.p, &g.q, &g.r);

      uint8_t tmp[8];
      memcpy(tmp, i2c_trans.buf, 8);
      DOWNLINK_SEND_DEBUG(DefaultChannel, DefaultDevice, 8, tmp);


    });
    //    DEBUG_S5_OFF();
  }
}

static inline void main_init_hw( void ) {
#warning "Needs to be ported to libopencm3 or use the real driver!"

#if 0
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
#endif

}


void exti15_10_irq_handler(void) {

  //  DEBUG_S4_ON();

  /* clear EXTI */
  exti_reset_request(EXTI14);

  //  DEBUG_S4_TOGGLE();

  if (gyro_state == INITIALIZED) gyro_ready_for_read = TRUE;

  //  DEBUG_S4_OFF();

}
