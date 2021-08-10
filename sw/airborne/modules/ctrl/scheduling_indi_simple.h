/*
 * Copyright (C) 2017 Ewoud Smeur <ewoud_smeur@msn.com>
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
 * @file modules/ctrl/scheduling_indi_simple.h
 */

#ifndef SCHEDULING_INDI_SIMPLE_H
#define SCHEDULING_INDI_SIMPLE_H

#include "generated/airframe.h"
#include "firmwares/rotorcraft/stabilization/stabilization_indi_simple.h"

extern int32_t use_scheduling;

/**
 * Initialises periodic loop;
 */
extern void ctrl_eff_scheduling_init(void);

/**
 * Periodic function that interpolates between gain sets depending on the scheduling variable.
 */
extern void ctrl_eff_scheduling_periodic(void);

#endif  /* SCHEDULING_INDI_SIMPLE_H */

