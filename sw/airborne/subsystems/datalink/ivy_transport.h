/*
 * Copyright (C) 2003  Pascal Brisset, Antoine Drouin
 * Copyright (C) 2014  Gautier Hattenberger <gautier.hattenberger@enac.fr>
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
 * @file subsystems/datalink/ivy_transport.h
 *
 * Building Paparazzi frames over IVY.
 *
 */

#ifndef IVY_TRANSPORT_H
#define IVY_TRANSPORT_H

#include "subsystems/datalink/transport.h"
#include "mcu_periph/link_device.h"

// IVY transport
struct ivy_transport {
  char ivy_buf[256];
  char *ivy_p;
  int ivy_dl_enabled;
  // generic transmission interface
  struct transport_tx trans_tx;
  // generic (dummy) device
  struct link_device device;
};

extern struct ivy_transport ivy_tp;

// Init function
extern void ivy_transport_init(void);

#endif // IVY_TRANSPORT_H

