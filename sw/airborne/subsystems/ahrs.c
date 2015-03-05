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
 * @file subsystems/ahrs.c
 * Dispatcher to register actual AHRS implementations.
 */


#include "subsystems/ahrs.h"

#ifndef DefaultAhrsImpl
#warning "DefaultAhrsImpl not set!"
#else
PRINT_CONFIG_VAR(DefaultAhrsImpl)
#endif

#define __DefaultAhrsRegister(_x) _x ## _register()
#define _DefaultAhrsRegister(_x) __DefaultAhrsRegister(_x)
#define DefaultAhrsRegister() _DefaultAhrsRegister(DefaultAhrsImpl)

/** Attitude and Heading Reference System state */
struct Ahrs {
  /* function pointers to actual implementation, set by ahrs_register_impl */
  AhrsInit init;
};

struct Ahrs ahrs;

void ahrs_register_impl(AhrsInit init)
{
  ahrs.init = init;

  ahrs.init();
}

void ahrs_init(void)
{
  ahrs.init = NULL;
#ifdef DefaultAhrsImpl
  DefaultAhrsRegister();
#endif
}
