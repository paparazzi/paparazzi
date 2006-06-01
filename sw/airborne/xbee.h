/*
 * $Id$
 *  
 * Copyright (C) 2006  Pascal Brisset, Antoine Drouin
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

/* Maxstream XBee serial input and output */

#include "datalink.h"

#ifndef XBEE_H
#define XBEE_H

#define __XBeeLink(dev, _x) dev##_x
#define _XBeeLink(dev, _x)  __XBeeLink(dev, _x)
#define XBeeLink(_x) _XBeeLink(XBEE_UART, _x)

#define XBeeBuffer() XBeeLink(ChAvailable())
#define ReadXBeeBuffer() { while (XBeeLink(ChAvailable())&&!wc_msg_received) parse_xbee(XBeeLink(Getch())); }

#define XBeePrintString(s) XBeeLink(PrintString(s))
#define XBeePrintHex16(x) XBeeLink(PrintHex16(x))

/** Using "Transparent Operation" mode, the messages have to be packed with the pprz
    protocol */
#if DATALINK == XBEE
#define PPRZ_UART XBEE_UART
#endif

void xbee_init( void );

#endif /* XBEE_H */
