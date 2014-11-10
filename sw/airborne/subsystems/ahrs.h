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
 * Attitude and Heading Reference System interface.
 */

#ifndef AHRS_H
#define AHRS_H

#include "std.h"

/* underlying includes (needed for parameters) */
#ifdef AHRS_TYPE_H
#include AHRS_TYPE_H
#endif

typedef void (*AhrsInit)(void);
typedef void (*AhrsUpdateGps)(void);

/** Attitude and Heading Reference System state */
struct Ahrs {
  /* function pointers to actual implementation, set by ahrs_register_impl */
  AhrsInit init;
  AhrsUpdateGps update_gps;
};

/** global AHRS state */
extern struct Ahrs ahrs;

extern void ahrs_register_impl(AhrsInit init, AhrsUpdateGps update_gps);

/** AHRS initialization. Called at startup.
 * Initialized the global AHRS struct.
 */
extern void ahrs_init(void);

/** Update AHRS state with GPS measurements.
 *  Calls implementation if registered.
 *  Reads the global #gps data struct.
 */
extern void ahrs_update_gps(void);

#endif /* AHRS_H */
