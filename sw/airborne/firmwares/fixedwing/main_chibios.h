/*
 * Copyright (C) 2015 AggieAir, A Remote Sensing Unmanned Aerial System for Scientific Applications
 * Utah State University, http://aggieair.usu.edu/
 *
 * Michal Podhradsky (michal.podhradsky@aggiemail.usu.edu)
 * Calvin Coopmans (c.r.coopmans@ieee.org)
 *
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
 * @file main_chibios.h
 * Main file for ChibiOS/RT Paparazzi fixedwing
 *
 * Includes both Paparazzi and ChibiOS files, threads are static.
 *
 * @author {Michal Podhradsky, Calvin Coopmans}
 */
#ifndef MAIN_CHIBIOS_H
#define MAIN_CHIBIOS_H

/* ChibiOS includes */
#include "ch.h"

#include "firmwares/fixedwing/main_fbw.h"
#include "firmwares/fixedwing/main_ap.h"

/* Paparazzi includes */
#include "mcu.h" // mcu_init() function
#include "led.h" // LED_TOGGLE macros
#include "subsystems/actuators.h" // actuators
#include "firmwares/fixedwing/main_ap.h" // Autopilot tasks
#include "mcu_periph/sys_time.h" // sys time
#include "subsystems/radio_control.h" // radio event/periodic
#include "subsystems/electrical.h" // electrical periodic


/* Telemetry includes for CHIBIOS_INFO */
#if PERIODIC_TELEMETRY
#include "subsystems/datalink/telemetry.h"
#endif

#ifdef FBW_DATALINK
#include "firmwares/fixedwing/fbw_datalink.h"
#endif

/* if PRINT_CONFIG is defined, print some config options */
PRINT_CONFIG_VAR(PERIODIC_FREQUENCY)

/*
 *  TELEMETRY_FREQUENCY is defined in generated/periodic_telemetry.h
 * defaults to 60Hz or set by TELEMETRY_FREQUENCY configure option in airframe file
 */
PRINT_CONFIG_VAR(TELEMETRY_FREQUENCY)

/*
 * MODULES_FREQUENCY is defined in generated/modules.h
 * according to main_freq parameter set for modules in airframe file
 */
PRINT_CONFIG_VAR(MODULES_FREQUENCY)

#ifndef BARO_PERIODIC_FREQUENCY
#define BARO_PERIODIC_FREQUENCY 100
#endif
PRINT_CONFIG_VAR(BARO_PERIODIC_FREQUENCY)

#ifndef FAILSAFE_FREQUENCY
#define FAILSAFE_FREQUENCY 20
#endif
PRINT_CONFIG_VAR(FAILSAFE_FREQUENCY)

#ifndef ELECTRICAL_PERIODIC_FREQ
#define ELECTRICAL_PERIODIC_FREQ 10
#endif
PRINT_CONFIG_VAR(ELECTRICAL_PERIODIC_FREQ)

#ifndef RADIO_CONTROL_FREQ
#define RADIO_CONTROL_FREQ 60
#endif
PRINT_CONFIG_VAR(RADIO_CONTROL_FREQ)

#ifndef MONITOR_FREQUENCY
#define MONITOR_FREQUENCY 1
#endif
PRINT_CONFIG_VAR(MONITOR_FREQUENCY)

#ifndef  SYS_TIME_FREQUENCY
#error SYS_TIME_FREQUENCY should be defined in Makefile.chibios or airframe.xml and be equal to CH_CFG_ST_FREQUENCY
#elif SYS_TIME_FREQUENCY != CH_CFG_ST_FREQUENCY
#error SYS_TIME_FREQUENCY should be equal to CH_CFG_ST_FREQUENCY
#elif  CH_CFG_ST_FREQUENCY < (2 * PERIODIC_FREQUENCY)
#error CH_CFG_ST_FREQUENCY and SYS_TIME_FREQUENCY should be >= 2 x PERIODIC_FREQUENCY
#endif




#endif /* MAIN_CHIBIOS_H */
