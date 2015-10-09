/*
 * Copyright (C) 2015
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

/**
 * @file exif_module.h
 *
 * Write JPEG images containing EXIF headers with GPS coordinates.
 *
 */



int write_exif_jpeg(char *filename, const unsigned char *image_jpg, const unsigned int image_jpg_len,
                    const unsigned int image_jpg_x, const unsigned int image_jpg_y);


void push_gps_to_vision(void);

