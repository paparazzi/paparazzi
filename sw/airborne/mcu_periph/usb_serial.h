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
#include "pprzlink/pprzlink_device.h"

struct usb_serial_periph {
  /** Generic device interface */
  struct link_device device;
};

extern struct usb_serial_periph usb_serial;

void VCOM_init(void);
int  VCOM_putchar(int c);
int  VCOM_getchar(void);
int VCOM_peekchar(int ofs);
bool VCOM_check_free_space(uint16_t len);
int VCOM_check_available(void);
void VCOM_set_linecoding(uint8_t mode);
void VCOM_allow_linecoding(uint8_t mode);
void VCOM_send_message(void);
void VCOM_event(void);

#endif /* USB_S_H */
