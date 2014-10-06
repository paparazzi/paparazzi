/*
 * Copyright (C) 2003-2014 The Paparazzi Team
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
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */

/**
 * @file link_mcu.h
 * Common transport functions for the communication between FBW and AP.
 */

#ifndef LINK_MCU_H
#define LINK_MCU_H

#ifdef MCU_SPI_LINK
#include "link_mcu_spi.h"
#elif defined MCU_UART_LINK
#include "link_mcu_usart.h"
#elif defined MCU_CAN_LINK
#include "link_mcu_can.h"
#endif

#endif /* LINK_MCU_H */
