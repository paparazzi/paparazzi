/*
 * Copyright (C) 2013  Elisabeth van der Sman, 2013 Freek van Tienen
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
 * @file modules/time/time_countdown.h
 *
 * Count down remaining time.
 * Set an initial countdown value in seconds (re-settable via settings/GCS)
 * and countdown @a time_until_end variable to zero.
 * E.g. allows to check how much time is left before the end of the competition.
 */

#ifndef TIME_COUNTDOWN_H
#define TIME_COUNTDOWN_H

#include "std.h"

extern uint16_t time_until_end;

void time_countdown_init(void);
void time_countdown_periodic_1hz(void);

#endif /* TIME_COUNTDOWN_H */
