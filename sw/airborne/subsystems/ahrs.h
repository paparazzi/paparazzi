/*
 * Copyright (C) 2008-2010 The Paparazzi Team
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
 * @file subsystems/ahrs.h
 * Dispatcher to register actual AHRS implementations.
 */

#ifndef AHRS_H
#define AHRS_H

#include "std.h"

/* include actual (primary) implementation header */
#ifdef AHRS_TYPE_H
#include AHRS_TYPE_H
#endif

/* include secondary implementation header */
#ifdef AHRS_SECONDARY_TYPE_H
#include AHRS_SECONDARY_TYPE_H
#endif

typedef bool_t (*AhrsEnableOutput)(bool_t);
typedef bool_t (*AhrsIsAligned)(void);

/* for settings when using secondary AHRS */
extern uint8_t ahrs_output_idx;

/**
 * Register an AHRS implementation.
 * Adds it to an internal list.
 * @param enable pointer to function to enable/disable the output of registering AHRS
 */
extern void ahrs_register_impl(AhrsEnableOutput enable, AhrsIsAligned aligned);

/** AHRS initialization. Called at startup.
 * Registers/initializes the default AHRS.
 */
extern void ahrs_init(void);

/**
 * Switch to the output of another AHRS impl.
 * @param idx index of the AHRS impl (0 = PRIMARY_AHRS, 1 = SECONDARY_AHRS).
 */
extern int ahrs_switch(uint8_t idx);

/** AHRS initialization check. Returns if all registered AHRS implementations are running.
 */
extern bool_t ahrs_all_aligned(void);

#endif /* AHRS_H */
