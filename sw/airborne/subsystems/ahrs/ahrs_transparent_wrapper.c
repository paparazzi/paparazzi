/*
 * Copyright (C) 2014 Felix Ruess <felix.ruess@gmail.com>
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
 * @file subsystems/ahrs/ahrs_transparent_wrapper.c
 *
 * Paparazzi specific wrapper to run transparent AHRS
 */

#include "subsystems/ahrs/ahrs_transparent_wrapper.h"
#include "subsystems/ahrs.h"

#ifndef AHRS_TRANSPARENT_OUTPUT_ENABLED
#define AHRS_TRANSPARENT_OUTPUT_ENABLED TRUE
#endif
PRINT_CONFIG_VAR(AHRS_TRANSPARENT_OUTPUT_ENABLED)

/** if TRUE with push the estimation results to the state interface */
static bool_t ahrs_transparent_output_enabled;

static bool_t ahrs_transparent_enable_output(bool_t enable)
{
  ahrs_transparent_output_enabled = enable;
  return ahrs_transparent_output_enabled;
}

void ahrs_transparent_register(void)
{
  ahrs_transparent_output_enabled = AHRS_TRANSPARENT_OUTPUT_ENABLED;
  ahrs_transparent_init();
  ahrs_register_impl(ahrs_transparent_enable_output);
}

