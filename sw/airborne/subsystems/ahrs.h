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

/* underlying includes (needed for parameters) */
#ifdef AHRS_TYPE_H
#include AHRS_TYPE_H
#endif

typedef void (*AhrsInit)(void);

extern void ahrs_register_impl(AhrsInit init);

/** AHRS initialization. Called at startup.
 * Initialized the global AHRS struct.
 */
extern void ahrs_init(void);

#endif /* AHRS_H */
