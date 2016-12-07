/*
 * Copyright (C) Freek van Tienen
 *
 * This file is part of paparazzi
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
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */
/**
 * @file "modules/switching/switch_uart.h"
 * @author Freek van Tienen
 * Module for dropping balls using UART
 */

#ifndef SWITCH_UART_H
#define SWITCH_UART_H

#include "std.h"

#define switch_uart_SwitchUartChannel(X) ({switch_uart_channel=X;drop_ball(switch_uart_channel); false;})

extern uint8_t switch_uart_channel;
extern uint8_t switch_uart_status;

extern void drop_ball(uint8_t number);

extern void periodic_switch_uart(void);

#endif

