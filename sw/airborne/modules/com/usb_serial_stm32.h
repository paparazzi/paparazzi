/*
 * Copyright (C) 2014 Michal Podhradsky
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

/**
 * @file modules/com/usb_serial_stm32.h
 * header for serial over USB modules
 */

#ifndef USB_SERIAL_STM32_H
#define USB_SERIAL_STM32_H

#include "mcu_periph/usb_serial.h"

void init_usb_serial(void);
void periodic_usb_serial(void);
void event_usb_serial(void);

void usb_serial_parse_packet(int c);

#endif // USB_SERIAL_STM32_H
