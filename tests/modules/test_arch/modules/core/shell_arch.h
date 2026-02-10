/*
 * Copyright (C) Alexandre Bustico <alexandre.bustico@enac.fr>
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

/** @file "modules/core/shell_arch.h"
 * Dummy test file
 */

#ifndef SHELL_ARCH_H
#define SHELL_ARCH_H

typedef void shell_stream_t;
extern void shell_init_arch(void);

extern void chprintf(shell_stream_t *lchp, const char *fmt, ...) ;

#endif  // SHELL_ARCH_H
