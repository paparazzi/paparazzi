/*
 * $Id$
 *
 * Copyright (C) 2003  Pascal Brisset, Antoine Drouin
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

/** \file met_module.h
 *  \brief Device independent serial meteo code
 *
*/


#ifndef MET_H
#define MET_H

#include "std.h"
#include "led.h"

#ifndef SITL
#include "mcu_periph/uart.h"

#define MetLinkDevice (&(MET_LINK).device)

#define MetBuffer() MetLinkDevice->char_available(MetLinkDevice->periph)
#define MetGetch() MetLinkDevice->get_byte(MetLinkDevice->periph)
#define ReadMetBuffer() { while (MetBuffer()&&!met_msg_received) parse_met_buffer(MetGetch()); }
#define MetSend1(c) MetLinkDevice->put_byte(MetLinkDevice->periph, c)
#define MetUartSend1(c) MetSend1(c)
#define MetSend(_dat,_len) { for (uint8_t i = 0; i< (_len); i++) MetSend1(_dat[i]); };
#define MetUartSetBaudrate(_b) uart_periph_set_baudrate(&(MET_LINK), _b)
#define MetUartRunning (MET_LINK).tx_running

#endif /** !SITL */

#endif /* MET_H */

