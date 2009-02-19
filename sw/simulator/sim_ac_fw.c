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

#include "sim_ac_jsbsim.hpp"


#include "main_ap.h"
#include "main_fbw.h"

static void sim_gps_feed_data(void);
static void sim_ir_feed_data(void);

void autopilot_init(void) {
  init_fbw();
  init_ap();
}

void autopilot_periodic_task(void) {
  periodic_task_ap();
  periodic_task_fbw();
}

void autopilot_event_task(void) {
  event_task_ap();
  event_task_fbw();
}


void copy_inputs_to_jsbsim(JSBSim::FGFDMExec & FDMExec) {
}

void copy_outputs_from_jsbsim(JSBSim::FGFDMExec & FDMExec) {
}

#include "gps.h"
static void sim_gps_feed_data(void) {
}

