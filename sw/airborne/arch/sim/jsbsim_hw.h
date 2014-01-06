/*
 *
 * Copyright (C) 2009 Enac
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

/** \file jsbsim_hw.h
 */

#ifndef JSBSIM_HW_H
#define JSBSIM_HW_H

#include <inttypes.h>
#include "std.h"
#include "inter_mcu.h"
#include "firmwares/fixedwing/autopilot.h"
#include "subsystems/gps.h"
#include "subsystems/navigation/traffic_info.h"
#include "generated/flight_plan.h"
#include "generated/airframe.h"
#include "generated/settings.h"
#include "subsystems/nav.h"
#include "firmwares/fixedwing/stabilization/stabilization_attitude.h"
#include "firmwares/fixedwing/guidance/guidance_v.h"
#include "subsystems/sensors/infrared.h"
#include "subsystems/commands.h"
#include "firmwares/fixedwing/main_ap.h"
#include "subsystems/datalink/downlink.h"
#include "sim_uart.h"
#include "subsystems/datalink/datalink.h"


void sim_use_gps_pos(double lat, double lon, double alt, double course, double gspeed, double climb, double time);
void sim_update_sv(void);

void set_ir(double roll, double pitch);
void provide_attitude_and_rates(float phi, float theta, float psi, float p, float q, float r);

void update_bat(double bat);

void parse_dl_ping(char* argv[]);
void parse_dl_acinfo(char* argv[]);
void parse_dl_setting(char* argv[]);
void parse_dl_get_setting(char* argv[]);
void parse_dl_block(char* argv[]);
void parse_dl_move_wp(char* argv[]);

#endif
