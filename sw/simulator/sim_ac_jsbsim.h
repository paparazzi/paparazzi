/*
 * Copyright (C) 2008 Gautier Hattenberger
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

#ifndef SIM_AC_JSBSIM_H
#define SIM_AC_JSBSIM_H

// ignore stupid warnings in JSBSim
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#include <FGFDMExec.h>
#include <FGJSBBase.h>
#include <models/FGPropulsion.h>
#pragma GCC diagnostic pop

#include "std.h"
#include "generated/airframe.h"
#include "generated/flight_plan.h"
#include "mcu_periph/sys_time.h"

#include <Ivy/ivy.h>

#ifndef JSBSIM_PERIOD
#define JSBSIM_SPEEDUP 4 ///< how many JSBSim calls per A/P control loop call?
#define JSBSIM_PERIOD (1000.0/CONTROL_FREQUENCY/JSBSIM_SPEEDUP) ///< JSBSim timestep in milliseconds
#else
#define JSBSIM_SPEEDUP ((uint8_t) (1000./CONTROL_FREQUENCY/JSBSIM_PERIOD))
#endif
#define DT (JSBSIM_PERIOD*1e-3) ///< JSBSim timestep in seconds

#define SYSTIME_PERIOD ((uint32_t)(1000. / SYS_TIME_FREQUENCY)) ///< in msec

#define RAD2DEG 57.29578
#define FT2M 0.3048

extern bool run_model;

void sim_autopilot_init(void);
void autopilot_periodic_task(void);
void autopilot_event_task(void);
void jsbsim_init(void);
void copy_inputs_to_jsbsim(JSBSim::FGFDMExec* FDMExec);
void copy_outputs_from_jsbsim(JSBSim::FGFDMExec* FDMExec);
bool check_crash_jsbsim(JSBSim::FGFDMExec* FDMExec);


#endif // SIM_AC_JSBSIM_H
