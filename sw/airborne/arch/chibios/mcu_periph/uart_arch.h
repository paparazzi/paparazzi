/*
 * Copyright (C) 2013 AggieAir, A Remote Sensing Unmanned Aerial System for Scientific Applications
 * Utah State University, http://aggieair.usu.edu/
 *
 * Michal Podhradsky (michal.podhradsky@aggiemail.usu.edu)
 * Calvin Coopmans (c.r.coopmans@ieee.org)
 *
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
/**
 * @brief chibios arch dependant implementation of uart/serial drivers
 * @details Partially implemented (no error callbacks yet). Since ChibiOs
 * 			has SerialDriver, it is better to use that one instead of Uart
 * 			drivers, although it makes backward compatibility more difficutl.
 * 			DMA is already implemented.
 */
#ifndef CHIBIOS_UART_ARCH_H
#define CHIBIOS_UART_ARCH_H

#define B1200    1200
#define B2400    2400
#define B4800    4800
#define B9600    9600
#define B19200   19200
#define B38400   38400
#define B57600   57600
#define B100000  100000
#define B115200  115200
#define B230400  230400
#define B921600  921600

#endif /* STM32_UART_ARCH_H */
