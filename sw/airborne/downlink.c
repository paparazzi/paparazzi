/*
 * Paparazzi $Id$
 *
 * Copyright (C) 2006 Pascal Brisset, Antoine Drouin
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

/** \file downlink.c
 *  \brief Common code for AP and FBW telemetry
 *
 */


#include "std.h"
#include "generated/airframe.h"

#ifdef FBW
#ifndef TELEMETRY_MODE_FBW
#define TELEMETRY_MODE_FBW 0
#endif
uint8_t telemetry_mode_Fbw_DefaultChannel = TELEMETRY_MODE_FBW;
#endif /** FBW */

#ifdef AP
#ifndef TELEMETRY_MODE_AP
#define TELEMETRY_MODE_AP 0
#endif
uint8_t telemetry_mode_Ap_DefaultChannel = TELEMETRY_MODE_AP;
#endif /** AP */

uint8_t downlink_nb_ovrn;
uint16_t downlink_nb_bytes;
uint16_t downlink_nb_msgs;
