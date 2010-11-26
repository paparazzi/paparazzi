/*
 * $Id$
 *
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

#include <FGFDMExec.h>
#include <FGJSBBase.h>
#include <models/FGPropulsion.h>

#include "std.h"
#include "generated/airframe.h"
#include "generated/flight_plan.h"

#include <Ivy/ivy.h>

/* 60Hz <-> 17ms */
#ifndef JSBSIM_PERIOD
#define JSBSIM_PERIOD 17
#endif
#define DT (JSBSIM_PERIOD*1e-3)

#define RAD2DEG 57.29578
#define FT2M 0.3048

extern bool run_model;

void autopilot_init(void);
void autopilot_periodic_task(void);
void autopilot_event_task(void);
void jsbsim_init(void);
void copy_inputs_to_jsbsim(JSBSim::FGFDMExec* FDMExec);
void copy_outputs_from_jsbsim(JSBSim::FGFDMExec* FDMExec);
bool check_crash_jsbsim(JSBSim::FGFDMExec* FDMExec);


#endif // SIM_AC_JSBSIM_H
