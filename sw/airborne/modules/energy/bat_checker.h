/*
 * Copyright (C) 2012  Thomas Kolb
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
 * @file modules/energy/bat_checker.c
 *
 * Activate a buzzer/LED periodically or periodically to warn of low/critical battery level.
 * At LOW_BAT_LEVEL the buzzer will be activated periodically.
 * At CRITIC_BAT_LEVEL the buzzer will be activated permanently.
 */

#ifndef BAT_CHECKER_H
#define BAT_CHECKER_H

void init_bat_checker(void);
void bat_checker_periodic(void);

#endif // BAT_CHECKER_H
