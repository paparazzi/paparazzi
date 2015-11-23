/*
 * Copyright (C) 2015 The Paparazzi Team
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

/** @file subsystems/intermcu/intermcu.h
 *  @brief Rotorcraft Inter-MCU interface
 */

#ifndef INTERMCU_ROTORCRAFT_H
#define INTERMCU_ROTORCRAFT_H

#include "std.h"
#include "subsystems/commands.h"

#define INTERMCU_AP   0
#define INTERMCU_FBW  1

#define INTERMCU_LOST_CNT 25  /* 50ms with a 512Hz timer TODO fixed value */

enum intermcu_status {
  INTERMCU_OK,
  INTERMCU_LOST
};

struct intermcu_t {
  enum intermcu_status status;
  uint8_t time_since_last_frame;
};
extern struct intermcu_t inter_mcu;

void intermcu_init(void);
void intermcu_periodic(void);

#endif
