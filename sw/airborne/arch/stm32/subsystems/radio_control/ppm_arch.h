/*
 * $Id$
 *
 * Copyright (C) 2010 The Paparazzi Team
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

/** \file booz_radio_control_ppm_hw.h
 *  \brief STM32 ppm decoder
 *
 */

#ifndef PPM_ARCH_H
#define PPM_ARCH_H

#include "mcu_periph/sys_time.h"

/**
 * On tiny (and booz) the ppm counter is running at the same speed as
 * the systic counter. There is no reason for this to be true.
 * Let's add a pair of macros to make it possible for them to be different.
 *
 */
#define RC_PPM_TICKS_OF_USEC(_v)        CPU_TICKS_OF_USEC((_v)/9)
#define RC_PPM_SIGNED_TICKS_OF_USEC(_v) SIGNED_CPU_TICKS_OF_USEC((_v)/9)
#define USEC_OF_RC_PPM_TICKS(_v)        USEC_OF_CPU_TICKS((_v)*9)

#define PPM_NB_CHANNEL RADIO_CONTROL_NB_CHANNEL

#endif /* PPM_ARCH_H */
