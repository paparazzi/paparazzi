/*
 * Copyright (C) 2011 The Paparazzi Team
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
 * @file subsystems/ahrs/ahrs_sim.h
 *
 * Interface to set the AHRS from the simple OCaml simulator.
 *
 */

#ifndef AHRS_SIM_H
#define AHRS_SIM_H

#include "subsystems/ahrs.h"
#include "std.h"

extern float ins_roll_neutral;
extern float ins_pitch_neutral;

extern void update_ahrs_from_sim(void);
extern void ahrs_sim_register(void);

#define PRIMARY_AHRS ahrs_sim

#endif /* AHRS_SIM_H */
