/*
 * Copyright (C) 2008-2012 The Paparazzi Team
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
 * along with Paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 */

/** @file supervision.h
 *  Supervision.
 *  Handles the mapping of roll/pitch/yaw commands
 *  to actual motor commands.
 */

#ifndef SUPERVISION_H
#define SUPERVISION_H

#include "std.h"
#include "generated/airframe.h"

struct Supervision {
  int32_t commands[SUPERVISION_NB_MOTOR];
  int32_t trim[SUPERVISION_NB_MOTOR];
  bool_t override_enabled[SUPERVISION_NB_MOTOR];
  int32_t override_value[SUPERVISION_NB_MOTOR];
  uint32_t nb_failure;
};

extern struct Supervision supervision;

extern void supervision_init(void);
extern void supervision_run(bool_t motors_on, bool_t override_on, int32_t in_cmd[]);
extern void supervision_run_spinup(uint32_t counter, uint32_t max_counter);

#endif /* SUPERVISION_H */
