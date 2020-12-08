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

/** @file "modules/core/shell.h"
 * @author Alexandre Bustico <alexandre.bustico@enac.fr>
 * Simple debug shell
 */

#ifndef SHELL_H
#define SHELL_H

#include "modules/core/shell_arch.h"

/** Init shell
 */
extern void shell_init(void);

/** Command handler
 */
typedef void (shell_cmd_t)(shell_stream_t *stream, int argc, const char * const argv[]);

/** Add new shell entry
 *
 * maximum number of entry might be limited, see specific implementations
 */
extern void shell_add_entry(char *cmd_name, shell_cmd_t *cmd);

#endif  // SHELL_H
