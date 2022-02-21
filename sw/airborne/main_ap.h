/*
 * Copyright (C) 2008-2021 The Paparazzi Team
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
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 *
 */

/**
 * @file main_ap.h
 *
 * Autopilot main loop.
 *
 * This process is reponsible for the collecting the different sensors data,
 * calling the appropriate estimation algorithms and running the different control loops.
 */

#ifndef MAIN_AP_H
#define MAIN_AP_H

extern void main_ap_init(void);
extern void main_ap_periodic(void);
extern void main_ap_event(void);

#endif /* MAIN_AP_H */

