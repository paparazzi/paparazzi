/*
 * Copyright (C) 2008  Antoine Drouin
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
 * @file arch/lpc21/mcu_periph/adc_arch.h
 * @ingroup lpc21_arch
 *
 * Handling of ADC hardware for lpc21xx.
 */

#ifndef ADC_ARCH_H
#define ADC_ARCH_H

#include BOARD_CONFIG

/* Set the correct ADC resolution */
#ifndef ADC_RESOLUTION
#define ADC_RESOLUTION 1024
#endif

/** 8 ADCs for bank 0, others for bank 2 */
#define NB_ADC 8

#define AdcBank0(x) (x)
#define AdcBank1(x) (x+NB_ADC)

#endif /* ADC_ARCH_H */
