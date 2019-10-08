/*
 * Copyright (C) 2019 Gautier Hattenberger <gautier.hattenberger@enac.fr>
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
 * @file "modules/computer_vision/cv_target_localization.h"
 * @author Gautier Hattenberger <gautier.hattenberger@enac.fr>
 *
 * Compute georeferenced position of a target from visual detection
 * assuming that the target is on a flat ground
 */

#ifndef CV_TARGET_LOCALIZATION_H
#define CV_TARGET_LOCALIZATION_H

#include "std.h"

extern void target_localization_init(void);
extern void target_localization_report(void);

// settings and handlers
extern uint8_t target_localization_mark;
extern void cv_target_localization_report_mark(uint8_t mark);
extern bool target_localization_update_wp;

// TODO move functionality to the camera driver
extern void target_localization_send_pos_to_cam(void);

#endif

