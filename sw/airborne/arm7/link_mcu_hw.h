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

#ifdef BOOZ_FILTER_MCU

#include "LPC21xx.h"
#define BoozLinkMcuSetUnavailable() { IO1SET = _BV(SPI0_DRDY); }
#define BoozLinkMcuSetAvailable()   { IO1CLR = _BV(SPI0_DRDY); }

#endif /* BOOZ_FILTER_MCU */





#endif /* LINK_MCU_HW_H */
