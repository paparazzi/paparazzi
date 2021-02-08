/*
 * Copyright (C) 2018 Kirk Scheper <kirkscheper@gmail.com>
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
 */

/** @file arch/linux/mcu_periph/pipe_arch.h
 * linux named pipe handling
 */

#ifndef PIPE_ARCH_H
#define PIPE_ARCH_H

// default pipe buffer sizes on linux
#ifndef PIPE_RX_BUFFER_SIZE
#define PIPE_RX_BUFFER_SIZE 1024
#endif
#ifndef PIPE_TX_BUFFER_SIZE
#define PIPE_TX_BUFFER_SIZE 1024
#endif

#endif /* PIPE_ARCH_H */
