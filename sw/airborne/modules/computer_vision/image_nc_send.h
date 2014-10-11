/*
 * Copyright (C) 2014 Gautier Hattenberger <gautier.hattenberger@enac.fr>
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
 * @file modules/computer_vision/image_nc_send.h
 *
 * Capture an image on an ARDrone2 and send it to the ground with netcat (nc)
 */

#ifndef IMAGE_NC_SEND_H
#define IMAGE_NC_SEND_H

// Module functions
extern void image_nc_send_run(void);
extern void image_nc_send_start(void);
extern void image_nc_send_stop(void);

#endif /* IMAGE_NC_SEND_H */

