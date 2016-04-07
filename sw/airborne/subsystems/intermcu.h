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
#include "pprzlink/pprz_transport.h"

#define INTERMCU_AP   0
#define INTERMCU_FBW  1

#ifndef INTERMCU_LOST_CNT
#define INTERMCU_LOST_CNT 25  /* 50ms with a 512Hz timer TODO fixed value */
#endif

#include BOARD_CONFIG

enum intermcu_status {
  INTERMCU_OK,
  INTERMCU_LOST
};

enum intermcu_PX4_baud_status {
  PX4_BAUD,
  CHANGING_BAUD,
  PPRZ_BAUD
};

struct intermcu_t {
  struct link_device *device;       ///< Device used for communication
  struct pprz_transport transport;  ///< Transport over communication line (PPRZ)
  enum intermcu_status status;      ///< Status of the INTERMCU
  uint8_t time_since_last_frame;    ///< Time since last frame
  bool enabled;                     ///< If the InterMCU communication is enabled
  bool msg_available;               ///< If we have an InterMCU message

#ifdef BOARD_PX4IO
  enum intermcu_PX4_baud_status stable_px4_baud;
#endif
};
extern struct intermcu_t intermcu;



void intermcu_init(void);
void intermcu_periodic(void);

#endif
