/*
 * $Id$
 *
 * Copyright (C) 2010 The Paparazzi Team
 *
 * This is the implementation of the "external interface" to the autopilot.
 * using overo_spi_link.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#ifndef VEHICLE_INTERFACE_OVERO_LINK_H
#define VEHICLE_INTERFACE_OVERO_LINK_H

#include "std.h"
#include "modules/vehicle_interface/vi.h"
#include "math/pprz_algebra_int.h"
#include "lisa/lisa_overo_link.h"

#define ViOveroLinkEvent() {						\
    OveroLinkEvent(vi_overo_link_on_msg_received, vi_overo_link_on_crc_err); \
  }

extern void vi_overo_link_on_msg_received(void);
extern void vi_overo_link_on_crc_err(void);

#endif /* VEHICLE_INTERFACE_OVERO_LINK_H */
