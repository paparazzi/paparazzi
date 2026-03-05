/*
 * Copyright (C) 2025 fab <fab@github.com>
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

/** @file "modules/sensors/thd_test.h"
 * @author fab <fab@github.com>
 * A module to test threads functionnalities, like threads, mutexes, semaphores and so on.
 */

#ifndef THD_TEST_H
#define THD_TEST_H

extern void thd_test_init(void);
extern void thd_test_periodic(void);
extern void thd_test_event(void);

void thd_test_syracuse_restart(float s);

extern float thd_test_start_value;

#endif  // THD_TEST_H
