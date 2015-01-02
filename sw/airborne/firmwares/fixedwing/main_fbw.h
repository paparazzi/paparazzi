/*
 * Copyright (C) 2005 Pascal Brisset, Antoine Drouin
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
 * @file firmwares/fixedwing/main_fbw.h
 *
 * FBW ( FlyByWire ) process API
 *
 */

#ifndef FBW_H
#define FBW_H

#include "std.h"

/** Fly by wire modes */
#define FBW_MODE_MANUAL   0
#define FBW_MODE_AUTO     1
#define FBW_MODE_FAILSAFE 2
#define FBW_MODE_OF_PPRZ(mode) ((mode) < THRESHOLD_MANUAL_PPRZ ? FBW_MODE_MANUAL : FBW_MODE_AUTO)

extern uint8_t fbw_mode;
extern bool_t failsafe_mode;

void init_fbw(void);
void handle_periodic_tasks_fbw(void);
void periodic_task_fbw(void);
void event_task_fbw(void);

#endif
