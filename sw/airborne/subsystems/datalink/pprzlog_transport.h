/*
 * Copyright (C) 2014 Gautier Hattenberger <gautier.hattenberger@enac.fr>
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
 *
 */

/**
 * @file subsystems/datalink/pprzlog_transport.h
 *
 * Protocol for on-board data logger with timestamp
 *
 */

#ifndef PPRZLOG_TRANSPORT_H
#define PPRZLOG_TRANSPORT_H

#include "mcu_periph/sys_time.h"
#include "subsystems/datalink/transport.h"

struct pprzlog_transport {
  // generic transmission interface
  struct transport_tx trans_tx;
  // specific pprz transport_tx variables
  uint8_t ck;
};

extern struct pprzlog_transport pprzlog_tp;

// Init function
extern void pprzlog_transport_init(void);

#endif

