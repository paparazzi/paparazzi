/*
 * Copyright (C) 2015 Michael Sierra <sierramichael.a@gmail.com>
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

/**
 * @file subsystems/gps/gps_multi.h
 *
 * multigps implementation
 */

#ifndef GPS_MULTI_H
#define GPS_MULTI_H

#include "subsystems/gps.h"

typedef void (*MultiGpsInit)(void);
typedef void (*MultiGpsEvent)(void);

/*
 * register callbacks and state pointers
 */ 
extern void gps_register_impl(MultiGpsInit init, MultiGpsEvent event, struct GpsState *gps_s, struct GpsTimeSync *timesync, int8_t instance);


#ifdef PRIMARY_GPS_TYPE_H
#include PRIMARY_GPS_TYPE_H
#endif
#ifdef SECONDARY_GPS_TYPE_H
#include SECONDARY_GPS_TYPE_H
#endif

#define GPS_MAX_INSTANCES 2
#define PRIMARY_GPS_INSTANCE 0
#define SECONDARY_GPS_INSTANCE 1

#define GPS_MODE_PRIMARY 0
#define GPS_MODE_SECONDARY 1
#define GPS_MODE_AUTO 2

#ifndef MULTI_GPS_MODE
#define MULTI_GPS_MODE GPS_MODE_AUTO
#endif

extern uint8_t multi_gps_mode;

void piksi_heartbeat(void);

void gps_multi_event(void);

/*
 * The GPS event
 */
#define GpsEvent gps_multi_event


#endif /* GPS_MULTI_H */