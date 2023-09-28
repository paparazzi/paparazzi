/*
 * Copyright (C) 2023 Gautier Hattenberger <gautier.hattenberger@enac.fr>
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
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */

/** @file modules/digital_cam/pprzlink_cam_ctrl.h
 *  @brief Digital Camera Control with PPRZLINK messages
 */

#ifndef PPRZLINK_CAM_CTRL_H
#define PPRZLINK_CAM_CTRL_H
#include "stdint.h"

extern void pprzlink_cam_ctrl_init(void);

/** Periodic */
extern void pprzlink_cam_ctrl_periodic(void);

/** Set expo setting */
extern float digital_cam_exposure;
extern void pprzlink_cam_ctrl_set_expo(float expo);
extern void dc_expo_cb(uint8_t* buf);
#define PPRZLINK_CAM_AUTO_EXPO 0.f

#endif // PPRZLINK_CAM_CTRL_H

