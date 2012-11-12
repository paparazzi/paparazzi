/*
 * Copyright (C) 2010 Eric Parsonage <eric@eparsonage.com>
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

#ifndef RADIO_CONTROL_SPEKTRUM_ARCH_H
#define RADIO_CONTROL_SPEKTRUM_ARCH_H


/*
 * All Spektrum and JR 2.4 GHz transmitters
 * have the same channel assignments.
 */


#ifndef RADIO_CONTROL_NB_CHANNEL
#define RADIO_CONTROL_NB_CHANNEL 12
#endif


#define RADIO_THROTTLE   0
#define RADIO_ROLL       1
#define RADIO_PITCH      2
#define RADIO_YAW        3
#define RADIO_GEAR       4
#define RADIO_FLAP       5
#define RADIO_AUX1       5
#define RADIO_AUX2       6
#define RADIO_AUX3       7
#define RADIO_AUX4       8
#define RADIO_AUX5       9
#define RADIO_AUX6       10
#define RADIO_AUX7       11

/* reverse some channels to suit Paparazzi conventions          */
/* the maximum number of channels a Spektrum can transmit is 12 */
#ifndef RADIO_CONTROL_SPEKTRUM_SIGNS
#define RADIO_CONTROL_SPEKTRUM_SIGNS {1,-1,-1,-1,1,-1,1,1,1,1,1,1}
#endif

/* really for a 9 channel transmitter
   we would swap the order of these */
#ifndef RADIO_MODE
#define RADIO_MODE       RADIO_GEAR
#endif

extern void RadioControlEventImp(void (*_received_frame_handler)(void));

#define UART1_RCC_GPIO RCC_APB2ENR_IOPAEN
#define UART1_RCC_REG &RCC_APB2ENR
#define UART1_RCC_DEV RCC_APB2ENR_USART1EN
#define UART1_BANK GPIO_BANK_USART1_RX
#define UART1_PIN GPIO_USART1_RX
#define UART1_IRQ NVIC_USART1_IRQ
#define UART1_ISR usart1_isr
#define UART1_DEV USART1
#define UART1_REMAP {}

#define UART3_RCC_GPIO RCC_APB2ENR_IOPCEN
#define UART3_RCC_REG &RCC_APB1ENR
#define UART3_RCC_DEV RCC_APB1ENR_USART3EN
#define UART3_BANK GPIO_BANK_USART3_PR_RX
#define UART3_PIN GPIO_USART3_PR_RX
#define UART3_IRQ NVIC_USART3_IRQ
#define UART3_ISR usart3_isr
#define UART3_DEV USART3
#define UART3_REMAP {AFIO_MAPR |= AFIO_MAPR_USART3_REMAP_PARTIAL_REMAP;}

#endif /* RADIO_CONTROL_SPEKTRUM_ARCH_H */
