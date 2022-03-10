/*
 * Copyright (C) 2013 Alexandre Bustico, Gautier Hattenberger
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
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */

/**
 * @file modules/radio_control/hott.h
 *
 * Radio control via single HOTT receiver in SUMD mode.
 */

#ifndef RC_HOTT_H
#define RC_HOTT_H

#include "modules/radio_control/hott_common.h"

extern struct SHott hott;

/**
 * RC init function.
 */
extern void hott_init(void);

/**
 * RC event function.
 */
extern void hott_event(void);


#endif /* RC_HOTT_H */
