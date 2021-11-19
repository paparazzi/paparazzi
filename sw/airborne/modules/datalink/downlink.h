/*
 * Copyright (C) 2003-2006  Pascal Brisset, Antoine Drouin
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

/** \file downlink.h
 *  \brief Common code for AP and FBW telemetry
 *
 */

#ifndef DOWNLINK_H
#define DOWNLINK_H

#include <inttypes.h>

#include "generated/airframe.h"
#include "pprzlink/messages.h"
#include "modules/datalink/datalink.h"

// FIXME test prog still need some includes here
#include "modules/datalink/pprz_dl.h"

#ifndef DefaultChannel
#define DefaultChannel DOWNLINK_TRANSPORT
#endif

#ifndef DefaultDevice
#define DefaultDevice DOWNLINK_DEVICE
#endif

// Init function
extern void downlink_init(void);

#include "generated/modules.h" // include at the end to avoid circular dependency FIXME

#endif /* DOWNLINK_H */
