/*
 * Copyright (C) 2006-2013 Pascal Brisset, Antoine Drouin, Gautier Hattenberger
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
 * @file firmwares/fixedwing/ap_downlink.h
 *
 * The periodic function is generated from the telemetry xml file.
 * Each subsystem of the autopilot can register function that
 * will be called if needed.
 *
 */

#ifndef AP_DOWNLINK_H
#define AP_DOWNLINK_H

#ifndef DOWNLINK_DEVICE
#define DOWNLINK_DEVICE DOWNLINK_AP_DEVICE
#endif
#include "subsystems/datalink/downlink.h"

#include "generated/periodic_telemetry.h"

#endif /* AP_DOWNLINK_H */
