/*
 * $Id$
 *
 * Copyright (C) 2012 Sergey Krukowski
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

#ifdef STM32F4
# include <libopencm3/stm32/f4/rcc.h>
# include <libopencm3/stm32/f4/gpio.h>
#include <libopencm3/stm32/f4/nvic.h>
#elif defined STM32F1
# include <libopencm3/stm32/f1/rcc.h>
# include <libopencm3/stm32/f1/gpio.h>
#include <libopencm3/stm32/f1/nvic.h>
#endif
#include <libopencm3/stm32/exti.h>


/*
 *               krooz
 * mag drdy      PA8
 *
 */

#include "subsystems/imu.h"
#include "subsystems/imu/imu_krooz1.h"
/**/
#include "subsystems/datalink/downlink.h"
#include "mcu_periph/uart.h"
#include "mcu_periph/sys_time.h"
#include "led.h"

uint8_t cnt;
uint32_t tick_last = 0;

struct ImuKrooz1 imu_krooz1;
bool_t periodic_flag;

#if defined(STM32F1) || defined(STM32F2)
extern void exti9_5_irq_handler(void);
#endif

static inline void krooz_init_hw( void ) {
#if defined(STM32F1)
	GPIO_InitTypeDef GPIO_InitStructure;
	EXTI_InitTypeDef EXTI_InitStructure;
	
	/* configure external interrupt exti8 on PA8( mag int ) */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	/* configure external interrupt exti5 on PB5( MPU6050 int ) */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource8);
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource5);
	
  EXTI_InitStructure.EXTI_Line = EXTI_Line8 | EXTI_Line5;
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
#elif defined(STM32F4)
	rcc_peripheral_enable_clock(&RCC_APB2ENR, RCC_APB2ENR_SYSCFGEN);
	rcc_peripheral_enable_clock(&RCC_AHB1ENR, RCC_AHB1ENR_IOPBEN);
#ifndef KROOZ_SD
	rcc_peripheral_enable_clock(&RCC_AHB1ENR, RCC_AHB1ENR_IOPAEN);
#else
  rcc_peripheral_enable_clock(&RCC_AHB1ENR, RCC_AHB1ENR_IOPCEN);
#endif
	gpio_mode_setup(GPIOB, GPIO_MODE_INPUT, GPIO_PUPD_NONE, GPIO5);
#ifndef KROOZ_SD
	gpio_mode_setup(GPIOA, GPIO_MODE_INPUT, GPIO_PUPD_NONE, GPIO8);
#else
  gpio_mode_setup(GPIOC, GPIO_MODE_INPUT, GPIO_PUPD_NONE, GPIO6);
#endif
	
	nvic_enable_irq(NVIC_EXTI9_5_IRQ);
	exti_select_source(EXTI5, GPIOB);
#ifndef KROOZ_SD
	exti_select_source(EXTI8, GPIOA);
#else
  exti_select_source(EXTI6, GPIOC);
#endif
  exti_set_trigger(EXTI5, EXTI_TRIGGER_FALLING);
#ifndef KROOZ_SD
	exti_set_trigger(EXTI8, EXTI_TRIGGER_FALLING);
#else
  exti_set_trigger(EXTI6, EXTI_TRIGGER_FALLING);
#endif
  exti_enable_request(EXTI5);
#ifndef KROOZ_SD
	exti_enable_request(EXTI8);
#else
  exti_enable_request(EXTI6);
#endif
  nvic_set_priority(NVIC_EXTI9_5_IRQ, 0x0F);
#endif
}

void imu_impl_init(void) {
  imu_krooz1.mpu_status = Krooz1StatusUninit;
	imu_krooz1.mag_status = Krooz1StatusUninit;
	periodic_flag = FALSE;
	krooz_init_hw();
}

#define I2CSetReg(_i2c_reg, _i2c_addr, _reg, _data, _status) { \
  _i2c_reg.type = I2CTransTx; \
	_i2c_reg.slave_addr = _i2c_addr; \
	_i2c_reg.buf[0] = _reg; \
	_i2c_reg.buf[1] = _data; \
	_i2c_reg.len_w = 2; \
	if(i2c_submit(&i2c2,&_i2c_reg)) \
	  _status++; \
}

static inline void mpu_config(void) {
  uint8_t data;
  if(imu_krooz1.mpu_trans.status == I2CTransSuccess || imu_krooz1.mpu_status == Krooz1StatusUninit) 
	{
	  switch (imu_krooz1.mpu_status) {
		  case 0:
			data = 0x01;
      I2CSetReg(imu_krooz1.mpu_trans, MPU60X0_I2C_ADDR, MPU60X0_REG_PWR_MGMT_1, data, imu_krooz1.mpu_status);
			//LED_ON(3)
		  break;
		  case 1:
			  data = (2 << 3) | 			// Fsync / ext sync on gyro X (bit 3->6)
						  (0 << 0);					// Low-Pass Filter
        I2CSetReg(imu_krooz1.mpu_trans, MPU60X0_I2C_ADDR, MPU60X0_REG_CONFIG, data, imu_krooz1.mpu_status);
		  break;
		  case 2:
		    data = 1;
        I2CSetReg(imu_krooz1.mpu_trans, MPU60X0_I2C_ADDR, MPU60X0_REG_SMPLRT_DIV, data, imu_krooz1.mpu_status);
		  break;
		  case 3:
		    data = (3 << 3);				// -2000deg/sec	
        I2CSetReg(imu_krooz1.mpu_trans, MPU60X0_I2C_ADDR, MPU60X0_REG_GYRO_CONFIG, data, imu_krooz1.mpu_status);
		  break;
		  case 4:
		    data = (0 << 0) |			// No HPFL
			  			 (3 << 3);			// Full Scale = 16g
        I2CSetReg(imu_krooz1.mpu_trans, MPU60X0_I2C_ADDR, MPU60X0_REG_ACCEL_CONFIG, data, imu_krooz1.mpu_status);
		  break;
		  case 5:
		    data = (1 << 4);			// Any read action clears INT status	
        I2CSetReg(imu_krooz1.mpu_trans, MPU60X0_I2C_ADDR, MPU60X0_REG_INT_PIN_CFG, data, imu_krooz1.mpu_status);
		  break;
		  case 6:
		    data = 1;		// INT enable	
        I2CSetReg(imu_krooz1.mpu_trans, MPU60X0_I2C_ADDR, MPU60X0_REG_INT_ENABLE, data, imu_krooz1.mpu_status);
			  imu_krooz1.mpu_status = Krooz1StatusIdle;
				//LED_OFF(3)
		  break;
		  default:
      break;
		}
	}
}

static inline void mag_config(void) {
  uint8_t data;
  if(imu_krooz1.mag_trans.status == I2CTransSuccess || imu_krooz1.mag_status == Krooz1StatusUninit) 
	{
	  switch (imu_krooz1.mag_status) {
		  case 0:
			data = 0x00 | (0x06 << 2);  // set to rate to 50Hz with HMC5843, 75Hz with HMC5883
      I2CSetReg(imu_krooz1.mag_trans, HMC5843_ADDR, HMC5843_REG_CFGA, data, imu_krooz1.mag_status);
			//LED_ON(2)
		  break;
		  case 1:
			  data = 0x01<<5;  // set to gain to 1 Gauss
        I2CSetReg(imu_krooz1.mag_trans, HMC5843_ADDR, HMC5843_REG_CFGB, data, imu_krooz1.mag_status);
		  break;
			case 2:
			  data = 0x00;  // set to continuous mode
        I2CSetReg(imu_krooz1.mag_trans, HMC5843_ADDR, HMC5843_REG_MODE, data, imu_krooz1.mag_status);
				imu_krooz1.mag_status = Krooz1StatusIdle;
				//LED_OFF(2)
		  break;
			default:
      break;
		}
	}
}

void imu_periodic( void ) {
	if(imu_krooz1.mpu_status < Krooz1StatusIdle) {
	  mpu_config();
		return;
	}
	if(imu_krooz1.mag_status < Krooz1StatusIdle) {
	  mag_config();
		return;
	}
	periodic_flag = TRUE;
/*	
	uint8_t buf[5];
	buf[0] = imu_krooz1.meas_nb;
	uint32_t time = SysTimeTimer(tick_last);
	//if(time > tick_last) 
	{
		SysTimeTimerStart(tick_last);
		buf[1] = (time >> 24)&0xFF;
		buf[2] = (time >> 16)&0xFF;//sys_time.nb_sec_rem - tick_last;
		buf[3] = (time >> 8)&0xFF;//sys_time.nb_sec_rem - tick_last;
		buf[4] = (time >> 0)&0xFF;//sys_time.nb_sec_rem - tick_last;
	}
	//else
	//	buf[0] = 0;
	cnt = 0;
	//RunOnceEvery(100,DOWNLINK_SEND_DEBUG(DefaultChannel, DefaultDevice, 5, buf));
*/
}
#if defined(STM32F1)
void exti9_5_irq_handler(void) {
  /* clear EXTI */
	if(EXTI_GetITStatus(EXTI_Line8) != RESET) {
    EXTI_ClearITPendingBit(EXTI_Line8);
		if (imu_krooz1.mag_status == Krooz1StatusIdle) 
	    imu_krooz1.mag_status = Krooz1StatusDataReady;
		//RunOnceEvery(10,LED_TOGGLE(2));
	}
  if(EXTI_GetITStatus(EXTI_Line5) != RESET) {
    EXTI_ClearITPendingBit(EXTI_Line5);
		if (imu_krooz1.mpu_status == Krooz1StatusIdle) 
	    imu_krooz1.mpu_status = Krooz1StatusDataReady;
		//RunOnceEvery(100,LED_TOGGLE(3));
		//cnt++;
	}
#elif defined(STM32F4)
void exti9_5_isr(void) {
	/* clear EXTI */
#ifndef KROOZ_SD
	if(EXTI_PR & EXTI8) {
    exti_reset_request(EXTI8);
#else
  if(EXTI_PR & EXTI6) {
    exti_reset_request(EXTI6);
#endif
		if (imu_krooz1.mag_status == Krooz1StatusIdle) 
	    imu_krooz1.mag_status = Krooz1StatusDataReady;
		//RunOnceEvery(10,LED_TOGGLE(2));
	}
	if(EXTI_PR & EXTI5) {
		exti_reset_request(EXTI5);
		if (imu_krooz1.mpu_status == Krooz1StatusIdle) 
			imu_krooz1.mpu_status = Krooz1StatusDataReady;
			//RunOnceEvery(100,LED_TOGGLE(3));
	}
}
#endif
