/*
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

#ifndef UART_ARCH_H
#define UART_ARCH_H

#include "mcu_periph/uart.h"
#include "std.h"

#define UartChAvailable(_p) uart_recieve(&_p)

#define B1200         1200
#define B2400         2400
#define B4800         4800
#define B9600         9600
#define B19200        19200
#define B38400        38400
#define B57600        57600
#define B115200       115200
#define B230400       230400

#endif /* UART_ARCH_H */
