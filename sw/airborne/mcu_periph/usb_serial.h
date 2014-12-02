/*
 * Copyright (C) 2006 Pascal Brisset, Antoine Drouin, Michel Gorraz
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

/** \file usb_serial.h
 *  \brief arch independent USB API
 *
 */

#ifndef USB_S_H
#define USB_S_H

#include <inttypes.h>
#include "std.h"
#include "mcu_periph/link_device.h"

struct usb_serial_periph {
  /** Generic device interface */
  struct link_device device;
};

extern struct usb_serial_periph usb_serial;

void VCOM_init(void);
int  VCOM_putchar(int c);
int  VCOM_getchar(void);
bool_t VCOM_check_free_space(uint8_t len);
int VCOM_check_available(void);
void VCOM_set_linecoding(uint8_t mode);
void VCOM_allow_linecoding(uint8_t mode);
void VCOM_transmit_message(void);
void VCOM_event(void);

/*
 * Macros can be used in subsystems that normally work with serial ports
 * e.g. use UsbS instead of UART1
 */
#define UsbSInit() VCOM_init()
#define UsbSCheckFreeSpace(_x) VCOM_check_free_space(_x)
#define UsbSTransmit(_x) VCOM_putchar(_x)
#define UsbSSendMessage() VCOM_transmit_message()
#define UsbSGetch() VCOM_getchar()
#define UsbSChAvailable() VCOM_check_available()

#endif /* USB_S_H */
