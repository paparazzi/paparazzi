/*
 * $Id:$
 *
 * Copyright (C) 2010 Piotr Esden-Tempski <piotr@esden.net>
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
 *
 */

#include <stdint.h>
#include <string.h>

#include "mcu_periph/can_arch.h"
#include "mcu_periph/can.h"

#include <stm32/rcc.h>
#include <stm32/gpio.h>
#include <stm32/flash.h>
#include <stm32/misc.h>
#include <stm32/can.h>

#include "led.h"

#define RCC_APB2Periph_GPIO_CAN RCC_APB2Periph_GPIOA
#define GPIO_CAN GPIOA
#define GPIO_Pin_CAN_RX GPIO_Pin_11
#define GPIO_Pin_CAN_TX GPIO_Pin_12

CanTxMsg can_tx_msg;
CanRxMsg can_rx_msg;

void _can_run_rx_callback(uint32_t id, uint8_t *buf, uint8_t len);

void can_hw_init(void)
{
	GPIO_InitTypeDef gpio;
	NVIC_InitTypeDef nvic;
	CAN_InitTypeDef can;
	CAN_FilterInitTypeDef can_filter;

	/* Enable peripheral clocks */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO |
			       RCC_APB2Periph_GPIOA, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);

	/* Configure CAN pin: RX */
	gpio.GPIO_Pin = GPIO_Pin_11;
	gpio.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_Init(GPIOA, &gpio);

	/* Configure CAN pin: TX */
	gpio.GPIO_Pin = GPIO_Pin_12;
	gpio.GPIO_Mode = GPIO_Mode_AF_PP;
	gpio.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &gpio);

	/* NVIC configuration */
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);

	nvic.NVIC_IRQChannel = USB_LP_CAN1_RX0_IRQn;
	nvic.NVIC_IRQChannelPreemptionPriority = 0x00;
	nvic.NVIC_IRQChannelSubPriority = 0x00;
	nvic.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&nvic);

	/* CAN register init */
	CAN_DeInit(CAN1);
	CAN_StructInit(&can);

	/* CAN cell init */
	can.CAN_TTCM = DISABLE;
	can.CAN_ABOM = CAN_ERR_RESUME;
	can.CAN_AWUM = DISABLE;
	can.CAN_NART = DISABLE;
	can.CAN_RFLM = DISABLE;
	can.CAN_TXFP = DISABLE;
	can.CAN_Mode = CAN_Mode_Normal;
	can.CAN_SJW = CAN_SJW_TQ;
	can.CAN_BS1 = CAN_BS1_TQ;
	can.CAN_BS2 = CAN_BS2_TQ;
	can.CAN_Prescaler = CAN_PRESCALER;
	can.CAN_ABOM = ENABLE;
	CAN_Init(CAN1, &can);

	/* CAN filter init */
	can_filter.CAN_FilterNumber = 0;
	can_filter.CAN_FilterMode = CAN_FilterMode_IdMask;
	can_filter.CAN_FilterScale = CAN_FilterScale_32bit;
	can_filter.CAN_FilterIdHigh = 0x0000;
	can_filter.CAN_FilterIdLow = 0x0000;
	can_filter.CAN_FilterMaskIdHigh = 0x0000;
	can_filter.CAN_FilterMaskIdLow = 0x0000;
	can_filter.CAN_FilterFIFOAssignment = 0;
	can_filter.CAN_FilterActivation = ENABLE;
	CAN_FilterInit(&can_filter);

	/* transmit struct init */
	can_tx_msg.StdId = 0x0;
	can_tx_msg.ExtId = 0x0;
	can_tx_msg.RTR = CAN_RTR_DATA;
#ifdef USE_CAN_EXT_ID
	can_tx_msg.IDE = CAN_ID_EXT;
#else
	can_tx_msg.IDE = CAN_ID_STD;
#endif
	can_tx_msg.DLC = 1;

	CAN_ITConfig(CAN1, CAN_IT_FMP0, ENABLE);
}

int can_hw_transmit(uint32_t id, const uint8_t *buf, uint8_t len)
{
	if(len > 8){
		return -1;
	}

#ifdef USE_CAN_EXT_ID
	can_tx_msg.ExtId = id;
#else
	can_tx_msg.StdId = id;
#endif
	can_tx_msg.DLC = len;

	memcpy(can_tx_msg.Data, buf, len);

	CAN_Transmit(CAN1, &can_tx_msg);

	return 0;
}

void usb_lp_can1_rx0_irq_handler(void)
{
	CAN_Receive(CAN1, CAN_FIFO0, &can_rx_msg);
#ifdef USE_CAN_EXT_ID
	_can_run_rx_callback(can_rx_msg.ExtId, can_rx_msg.Data, can_rx_msg.DLC);
#else
	_can_run_rx_callback(can_rx_msg.StdId, can_rx_msg.Data, can_rx_msg.DLC);
#endif
}

