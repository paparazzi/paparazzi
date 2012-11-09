/*
 * Copyright (C) 2012 Piotr Esden-Tempski <piotr@esden.net>
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

#include <libopencm3/stm32/f1/rcc.h>
#include <libopencm3/stm32/f1/gpio.h>
#include <libopencm3/stm32/can.h>
#include <libopencm3/cm3/nvic.h>

#include "led.h"

void _can_run_rx_callback(uint32_t id, uint8_t *buf, uint8_t len);

bool can_initialized = false;

void can_hw_init(void)
{

	/* Enable peripheral clocks. */
        rcc_peripheral_enable_clock(&RCC_APB2ENR, RCC_APB2ENR_AFIOEN);
        rcc_peripheral_enable_clock(&RCC_APB2ENR, RCC_APB2ENR_IOPBEN);
        rcc_peripheral_enable_clock(&RCC_APB1ENR, RCC_APB1ENR_CAN1EN);

	/* Remap the gpio pin if necessary. */
	AFIO_MAPR |= AFIO_MAPR_CAN1_REMAP_PORTB;

	/* Configure CAN pin: RX (input pull-up). */
	gpio_set_mode(GPIO_BANK_CAN1_PB_RX, GPIO_MODE_INPUT,
                      GPIO_CNF_INPUT_PULL_UPDOWN, GPIO_CAN1_PB_RX);
        gpio_set(GPIO_BANK_CAN1_PB_RX, GPIO_CAN1_PB_RX);

        /* Configure CAN pin: TX (output push-pull). */
        gpio_set_mode(GPIO_BANK_CAN1_PB_TX, GPIO_MODE_OUTPUT_50_MHZ,
                      GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO_CAN1_PB_TX);

	/* NVIC setup. */
        nvic_enable_irq(NVIC_USB_LP_CAN_RX0_IRQ);
        nvic_set_priority(NVIC_USB_LP_CAN_RX0_IRQ, 1);

	/* Reset CAN. */
        can_reset(CAN1);

	/* CAN cell init. */
        if (can_init(CAN1,
                     false,           /* TTCM: Time triggered comm mode? */
                     true,            /* ABOM: Automatic bus-off management? */
                     false,           /* AWUM: Automatic wakeup mode? */
                     false,           /* NART: No automatic retransmission? */
                     false,           /* RFLM: Receive FIFO locked mode? */
                     false,           /* TXFP: Transmit FIFO priority? */
                     CAN_BTR_SJW_1TQ,
                     CAN_BTR_TS1_3TQ,
                     CAN_BTR_TS2_4TQ,
                     12))             /* BRP+1: Baud rate prescaler */
        {
		/* TODO we need something somewhere where we can leave a note
		 * that CAN was unable to initialize. Just like any other
		 * driver should...
		 */

		can_reset(CAN1);

		return;
        }

        /* CAN filter 0 init. */
        can_filter_id_mask_32bit_init(CAN1,
                                0,     /* Filter ID */
                                0,     /* CAN ID */
                                0,     /* CAN ID mask */
                                0,     /* FIFO assignment (here: FIFO0) */
                                true); /* Enable the filter. */

        /* Enable CAN RX interrupt. */
        can_enable_irq(CAN1, CAN_IER_FMPIE0);

	/* Remember that we succeeded to initialize. */
	can_initialized = true;
}

int can_hw_transmit(uint32_t id, const uint8_t *buf, uint8_t len)
{

	if (!can_initialized) {
		return -2;
	}

	if(len > 8){
		return -1;
	}

	return can_transmit(CAN1,
			id,     /* (EX/ST)ID: CAN ID */
#ifdef USE_CAN_EXT_ID
			true,  /* IDE: CAN ID extended */
#else
			false, /* IDE: CAN ID not extended */
#endif
			false, /* RTR: Request transmit? */
			len,   /* DLC: Data length */
			buf);
}

void usb_lp_can_rx0_isr(void)
{
	u32 id, fmi;
        bool ext, rtr;
        u8 length, data[8];

	can_receive(CAN1,
		0,     /* FIFO: 0 */
		false, /* Release */
		&id,
		&ext,
		&rtr,
		&fmi,
		&length,
		data);

	_can_run_rx_callback(id, data, length);

	can_fifo_release(CAN1, 0);
}

