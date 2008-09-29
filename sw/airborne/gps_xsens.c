/*
 * $Id$
 *  
 * Copyright (C) 2005  Pascal Brisset, Antoine Drouin
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

/** \file gps_xsens.c
 *  \brief GPS hardware for Xsens MTi-G
 *
 */

#include <stdlib.h>

#include "gps.h"
#include "sys_time.h"
#include "airframe.h"

#include "uart.h"
#include "messages.h"
#include "downlink.h"

uint8_t gps_mode;
uint32_t gps_itow;
int32_t gps_alt;
uint16_t gps_gspeed;
int16_t gps_climb;
int16_t gps_course;
int32_t gps_utm_east, gps_utm_north;
uint8_t gps_utm_zone;
int32_t gps_lat, gps_lon;
uint16_t gps_PDOP;
uint32_t gps_Pacc, gps_Sacc;
uint8_t gps_numSV;
uint8_t gps_configuring;

uint16_t gps_reset;

uint16_t last_gps_msg_t;
bool_t gps_verbose_downlink;

volatile bool_t gps_msg_received;
bool_t gps_pos_available;
uint8_t gps_nb_ovrn;

uint8_t gps_nb_channels;

struct svinfo gps_svinfos[GPS_NB_CHANNELS];


void gps_init( void ) {}
void gps_configure( void ) {}
void parse_gps_msg( void ) {}
void gps_configure_uart( void ) {}

