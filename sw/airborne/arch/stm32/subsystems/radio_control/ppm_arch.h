/*
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

/**
 * @file arch/stm32/subsystems/radio_control/ppm_arch.h
 * @ingroup stm32_arch
 *
 * STM32 ppm decoder.
 *
 */

#ifndef PPM_ARCH_H
#define PPM_ARCH_H

#include "mcu_periph/sys_time.h"

/**
 * While the ppm counter is currently running at the same speed as
 * the systick counter, there is no reason for this to be true.
 * Let's add a pair of macros to make it possible for them to be different.
 */
#define RC_PPM_TICKS_OF_USEC(_v)        cpu_ticks_of_usec((_v))
#define RC_PPM_SIGNED_TICKS_OF_USEC(_v) (int32_t)((_v) * sys_time.cpu_ticks_per_sec * 1e-6)
#define USEC_OF_RC_PPM_TICKS(_v)        usec_of_cpu_ticks((_v))

#define PPM_NB_CHANNEL RADIO_CONTROL_NB_CHANNEL

#endif /* PPM_ARCH_H */
