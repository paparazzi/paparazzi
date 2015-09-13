/*
 * Copyright (C) 2015 Ewoud Smeur <ewoud.smeur@gmail.com>
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
 *
 */

/**
 * @file modules/exception_flags/exception_flags.h
 * @brief Flags to check if flight plan exceptions already triggered
 */

#include "std.h"

#ifndef EXCEPTION_FLAGS_H_
#define EXCEPTION_FLAGS_H_

extern void exception_flags_init(void);
extern void check_exception_flag(uint8_t flag_num);

extern bool_t exception_flag[10];

#endif /* EXCEPTION_FLAGS_H_ */