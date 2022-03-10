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

/** @file modules/intermcu/intermcu.h
 *  @brief Inter-MCU interface
 */

#ifndef INTERMCU_H
#define INTERMCU_H

#include "std.h"
#include "modules/core/commands.h"
#include "pprzlink/pprz_transport.h"

#ifndef INTERMCU_LOST_CNT
#define INTERMCU_LOST_CNT 25  /* 50ms with a 512Hz timer TODO fixed value */
#endif

#include BOARD_CONFIG

/* Different states the InterMCU can be in */
enum intermcu_status {
  INTERMCU_OK,                      ///< InterMCU communication is OK
  INTERMCU_LOST                     ///< No interMCU communication anymore
};

#ifdef BOARD_PX4IO
/* InterMCU baudrate protection for PX4 */
enum intermcu_PX4_baud_status {
  PX4_BAUD,
  CHANGING_BAUD,
  PPRZ_BAUD
};
#endif

/* InterMCU command status bits */
enum intermcu_cmd_status {
  INTERMCU_CMD_MOTORS_ON,           ///< The status of intermcu_ap_motors_on
  INTERMCU_CMD_DISARM,              ///< Whether or not to dis-arm the FBW
  INTERMCU_CMD_TIPPROPS,            ///< Enable tip props
  INTERMCU_CMD_FAILSAFE,            ///< Set FBW in failsafe mode
};

/* Easy accessible defines for cmd_status bits */
#define INTERMCU_SET_CMD_STATUS(_bit) { intermcu.cmd_status |= (1 << _bit);  }
#define INTERMCU_CLR_CMD_STATUS(_bit) { intermcu.cmd_status &= ~(1 << _bit); }
#define INTERMCU_GET_CMD_STATUS(_bit) (intermcu.cmd_status & (1 << _bit))

/* Main InterMCU structure */
struct intermcu_t {
  struct link_device *device;       ///< Device used for communication
  struct pprz_transport transport;  ///< Transport over communication line (PPRZ)
  enum intermcu_status status;      ///< Status of the INTERMCU
  uint8_t time_since_last_frame;    ///< Time since last frame
  bool enabled;                     ///< If the InterMCU communication is enabled
  bool msg_available;               ///< If we have an InterMCU message
  uint8_t cmd_status;               ///< Command status information that is transfered (intermcu_cmd_status)

#ifdef BOARD_PX4IO
  enum intermcu_PX4_baud_status stable_px4_baud;
#endif
};

extern struct intermcu_t intermcu;

/* Functions defined in XML */
extern void intermcu_init(void);
extern void intermcu_periodic(void);
extern void intermcu_event(void);

#endif

