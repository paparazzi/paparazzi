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

#ifndef SIM_AC_JSBSIM_HPP
#define SIM_AC_JSBSIM_HPP

#include <FGFDMExec.h>

#include "airframe.h"
#include "flight_plan.h"

void autopilot_init(void);
void autopilot_periodic_task(void);
void autopilot_event_task(void);
void copy_inputs_to_jsbsim(JSBSim::FGFDMExec & FDMExec);
void copy_outputs_from_jsbsim(JSBSim::FGFDMExec & FDMExec);


#endif // SIM_AC_JSBSIM_HPP
