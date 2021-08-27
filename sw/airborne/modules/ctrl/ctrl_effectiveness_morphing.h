/*
 * Copyright (C) 2021 Murat Bronz
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
 * @file modules/ctrl/ctrl_effectiveness_morphing.h
 */

#ifndef CTRL_EFFECTIVENESS_MORPHING_H
#define CTRL_EFFECTIVENESS_MORPHING_H

#include "generated/airframe.h"
#include "firmwares/rotorcraft/stabilization/stabilization_indi.h"

extern uint16_t adc_val1;
extern uint16_t adc_val2;

/**
 * Initialises periodic loop
 */
extern void ctrl_eff_morphing_init(void);

/**
 * Periodic function
 */
extern void ctrl_eff_morphing_periodic(void);
extern void ctrl_eff_morphing_periodic_a(void);

#endif  /* CTRL_EFFECTIVENESS_MORPHING_H */