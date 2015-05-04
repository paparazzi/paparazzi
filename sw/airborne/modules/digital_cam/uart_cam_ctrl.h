/*
 * Copyright (C) OpenUAS
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

/** @file modules/digital_cam/uart_cam_ctrl.h
 *  @brief Digital Camera Control Over UART with download of thumbnails over the PAYLOAD message
 *
 * Provides the control of a camera over serial, typically connected to a computer which has
 * full control of all camera functions via USB, including downloading of digital thumbnails
 *
 * The required initialization (digital_cam_uart_init()) and periodic process.
 */

#ifndef DIGITAL_CAM_UART_H
#define DIGITAL_CAM_UART_H

#include "modules/digital_cam/dc.h"

extern void digital_cam_uart_init(void);

extern void digital_cam_uart_periodic(void);
extern void digital_cam_uart_event(void);

extern int digital_cam_uart_thumbnails;
extern int digital_cam_uart_status;

#endif // GPIO_CAM_CTRL_H
