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
 *
 */

/**
 * @file modules/sensors/aoa_adc.c
 * @brief Angle of Attack sensor on ADC
 * Autor: Bruzzlee
 *
 * ex: US DIGITAL MA3-A10-236-N
 */

#ifndef AOA_ADC_H
#define AOA_ADC_H

#include "std.h"
#include "mcu_periph/adc.h"

/** Angle of Attack sensor via ADC.
 */
struct Aoa_Adc {
  struct adc_buf buf;
  uint32_t raw; ///< raw ADC value
  float angle;  ///< Angle of attack in radians
  float offset; ///< Angle of attack offset in radians
  float sens;   ///< sensitiviy, i.e. scale to conver raw to angle

  /** Filtering value [0-1]
   * 0: no filtering
   * 1: output is a constant value
   */
  float filter;
};

extern struct Aoa_Adc aoa_adc;

void aoa_adc_init(void);
void aoa_adc_update(void);

#endif /* AOA_ADC_H */
