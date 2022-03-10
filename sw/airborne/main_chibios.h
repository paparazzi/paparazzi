/*
 * Copyright (C) 2013-2021 Gautier Hattenberger, Alexandre Bustico
 *
 * This file is part of Paparazzi.
 *
 * Paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * Paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 *
 */

/**
 * @file main_chibios.h
 */

#ifndef MAIN_CHIBIOS_H
#define MAIN_CHIBIOS_H

#include <ch.h>

/** Terminate all autopilot threads
 *  Wait until proper stop
 */
extern void pprz_terminate_autopilot_threads(void);

#endif /* MAIN_CHIBIOS_H */
