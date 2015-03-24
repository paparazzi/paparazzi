/*
 * Copyright (C) 2015 The Paparazzi Community
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

/** @file modules/computer_vision/video_usb_logger.h
 *  @brief Camera image logger for Linux based autopilots
 */

#ifndef VIDEO_USB_LOGGER_H_
#define VIDEO_USB_LOGGER_H_

extern void video_usb_logger_start(void);
extern void video_usb_logger_stop(void);
extern void video_usb_logger_periodic(void);

#endif /* VIDEO_USB_LOGGER_H_ */
