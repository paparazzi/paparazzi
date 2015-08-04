/*
 * Copyright (C) 2009 Antoine Drouin <poinix@gmail.com>
 *
 * This file is part of Paparazzi.
 *
 * Paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * Paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 */

/** @file arch/linux/mcu_periph/uart_arch.h
 * linux uart handling
 */

#ifndef UART_ARCH_H
#define UART_ARCH_H

// higher default uart buffer sizes on linux
#ifndef UART_RX_BUFFER_SIZE
#define UART_RX_BUFFER_SIZE 512
#endif
#ifndef UART_TX_BUFFER_SIZE
#define UART_TX_BUFFER_SIZE 512
#endif

#include "mcu_periph/uart.h"

// for definition of baud rates
#include <termios.h>

// for conversion between linux baud rate definition and actual speed
static inline int uart_speed(int def)
{
  switch (def) {
    case B1200: return 1200;
    case B2400: return 2400;
    case B4800: return 4800;
    case B9600: return 9600;
    case B19200: return 19200;
    case B38400: return 38400;
    case B57600: return 57600;
    case B115200: return 115200;
    case B230400: return 230400;
    case B921600: return 921600;
    default: return 9600;
  }
}
#define UART_SPEED(_def) uart_speed(_def)

#endif /* UART_ARCH_H */
