/*
 * Copyright (C) 2008  Pascal Brisset, Antoine Drouin
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

/**
 * @file arch/lpc21/mcu_periph/dac_arch.h
 * @ingroup lpc21_arch
 *
 * Handling of DAC hardware for lpc21xx.
 */

#ifndef LPC21_MCU_PERIPH_DAC_ARCH_H
#define LPC21_MCU_PERIPH_DAC_ARCH_H

#include "std.h"
#include "LPC21xx.h"

static inline void DACSet(uint16_t x)
{
  DACR = x << 6;
}


#endif /* LPC21_MCU_PERIPH_DAC_ARCH_H */
