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

#ifndef IMU_B2_ARCH_H
#define IMU_B2_ARCH_H

/*

  MAX1168 SPI ADC connected on SPI1
  SS on P0.20
  EOC on P0.16 ( EINT0 )

  PNI mag on same bus
  SS on p1.28
  EOC P0.30 ( EINT3 )
  RESET P1.19

*/

#include "std.h"
#include "LPC21xx.h"
#include "interrupt_hw.h"

#define IMU_SSP_STA_IDLE           0
#define IMU_SSP_STA_BUSY_MAX1168   1
#define IMU_SSP_STA_BUSY_MS2100    2
extern volatile uint8_t imu_ssp_status;
extern int imu_overrun;





#endif /* IMU_B2_ARCH_H */
