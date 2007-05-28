/*
 * Paparazzi $Id$
 *  
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

/** \file uart.h
 *  \brief arch independant UART (Universal Asynchronous Receiver/Transmitter) API
 *
 */

#ifndef USB_S_H
#define USB_S_H

#include <inttypes.h>
#include "std.h"
//#include "usb_serial_hw.h"

void   usb_serial_init( void );
void   usb_serial_transmit( unsigned char data );
bool_t usb_serial_check_free_space( uint8_t len);

#define UsbSInit() usb_serial_init()
#define UsbSCheckFreeSpace(_x) usb_serial_check_free_space(_x)
#define UsbSTransmit(_x) usb_serial_transmit(_x)
#define UsbSSendMessage() {}

#endif /* USB_S_H */
