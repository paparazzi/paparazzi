/*
 * Copyright (C) 2015 The Paparazzi Team
 *
 * This file is part of Paparazzi.
 *
 * Paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * Paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 *
 */

/**
 * @file arch/chibios/modules/core/settings_arch.c
 * Persistent settings low level flash routines stm32.
 *
 * FIXME dummy file
 *
 */

#include "modules/core/settings.h"

int32_t persistent_write(void *ptr __attribute__((unused)), uint32_t size __attribute__((unused)))
{
  return -1;
}

int32_t persistent_read(void *ptr __attribute__((unused)), uint32_t size __attribute__((unused)))
{
  return 0;
}
