/*  $Id$
 *
 * Copyright (C) 2003-2005  Pascal Brisset, Antoine Drouin
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

/** \brief handling of arm7 inter mcu link
 *  
 */

#ifndef LINK_MCU_HW_H
#define LINK_MCU_HW_H


#define CRC_INIT 0x0
#define CrcLow(x) ((x)&0xff)
#define CrcHigh(x) ((x)>>8)

/*
#define CrcUpdate(_crc,_data) {       \
   uint8_t a = CrcHigh(_crc) + _data; \
   uint8_t b = CrcLow(_crc) + a;      \
   _crc = b | a >> 8;                 \
}
*/
#define CrcUpdate(_crc,_data) 0

#endif /* LINK_MCU_HW_H */
