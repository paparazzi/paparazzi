/*
 * $Id$
 *
 * Copyright (C) 2008-2009 Antoine Drouin <poinix@gmail.com>
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


#ifndef MAX1168_ARCH_H
#define MAX1168_ARCH_H


/*
  MAX1168 SPI ADC connected on SPI1
  SS on P0.20
  EOC on P0.16 ( EINT0 )
*/

#include "std.h"
#include "LPC21xx.h"
#include "interrupt_hw.h"
#include "ssp_hw.h"

#include "pprz_debug.h"

#define MAX1168_ERR_ISR_STATUS   0
#define MAX1168_ERR_READ_OVERUN  1
#define MAX1168_ERR_SPURIOUS_EOC 2

#define MAX1168_SS_PIN 20
#define MAX1168_SS_IODIR IO0DIR
#define MAX1168_SS_IOSET IO0SET
#define MAX1168_SS_IOCLR IO0CLR

#define MAX1168_EOC_PIN 16
#define MAX1168_EOC_PINSEL PINSEL1
#define MAX1168_EOC_PINSEL_BIT 0
#define MAX1168_EOC_PINSEL_VAL 1
#define MAX1168_EOC_EINT 0

#define Max1168Unselect() SetBit(MAX1168_SS_IOSET, MAX1168_SS_PIN)
#define Max1168Select() SetBit(MAX1168_SS_IOCLR, MAX1168_SS_PIN)

#define Max1168OnSpiInt() {						\
    ASSERT((max1168_status == STA_MAX1168_READING_RES),		\
       DEBUG_MAX_1168, MAX1168_ERR_ISR_STATUS);			\
    /* store convertion result */					\
    max1168_values[0] = SSPDR;					\
    max1168_values[1] = SSPDR;					\
    max1168_values[2] = SSPDR;					\
    max1168_values[3] = SSPDR;					\
    max1168_values[4] = SSPDR;					\
    max1168_values[5] = SSPDR;					\
    max1168_values[6] = SSPDR;					\
    max1168_values[7] = SSPDR;					\
    SSP_ClearRti();							\
    SSP_DisableRti();							\
    SSP_Disable();							\
    Max1168Unselect();							\
    max1168_status = STA_MAX1168_DATA_AVAILABLE;			\
  }

#endif /* MAX1168_ARCH_H */
