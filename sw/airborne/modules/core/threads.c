/*
 * Copyright (C) 2026 Freek van Tienen <freek.v.tienen@gmail.com>
 *
 * This file is part of paparazzi
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
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */

/**
 * @file "modules/core/threads.c"
 * Main thread of the modules, for pprz_thread_is_main() (ABI_CHECK_MAIN_THREAD)
 */

#include "modules/core/threads.h"

void pprz_thread_periodic(void)
{
  // The main thread is the one running the periodic functions (the init can run in another one, e.g. NPS)
  static bool main_set = false;
  if (!main_set) {
    pprz_thread_init_main();
    main_set = true;
  }
}
