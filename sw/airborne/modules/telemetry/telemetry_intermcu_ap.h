/*
 * Copyright (C) 2016 Freek van Tienen <freek.v.tienen@gmail.com>
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

/** @file modules/telemetry/telemetry_intermcu_ap.h
 *  @brief Telemetry through InterMCU
 */

#ifndef TELEMETRY_INTERMCU_AP_H
#define TELEMETRY_INTERMCU_AP_H

#include "std.h"
#include "pprzlink/short_transport.h"

/* Default maximum telemetry message size */
#ifndef TELEMERTY_INTERMCU_MSG_SIZE
#define TELEMERTY_INTERMCU_MSG_SIZE 128
#endif

/* Structure for handling telemetry over InterMCU */
struct telemetry_intermcu_t {
  struct link_device dev;                     ///< Device structure for communication
  struct short_transport trans;               ///< Transport without any extra encoding
  uint8_t buf[TELEMERTY_INTERMCU_MSG_SIZE];   ///< Buffer for the messages
  uint8_t buf_idx;                            ///< Index of the buffer
};

/* Telemetry InterMCU throughput */
extern struct telemetry_intermcu_t telemetry_intermcu;

#endif /* TELEMETRY_INTERMCU_AP_H */
