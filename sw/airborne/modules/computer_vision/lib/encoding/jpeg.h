/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 *  jpeg.c - JPEG compression for SRV-1 robot
 *    Copyright (C) 2005-2009  Surveyor Corporation
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details (www.gnu.org/licenses)
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

/**
 * @file modules/computer_vision/lib/encoding/jpeg.h
 * Encode images with the use of the JPEG encoding
 */

#ifndef _CV_ENCODING_JPEG_H
#define _CV_ENCODING_JPEG_H

#include "std.h"
#include "lib/vision/image.h"

/* The different type of image encodings */
#define FOUR_ZERO_ZERO          0
#define FOUR_TWO_ZERO           1
#define FOUR_TWO_TWO            2
#define FOUR_FOUR_FOUR          3
#define RGB                     4

/* JPEG encode an image */
void jpeg_encode_image(struct image_t *in, struct image_t *out, uint32_t quality_factor, bool_t add_dri_header);

/* Create an SVS header */
int jpeg_create_svs_header(unsigned char *buf, int32_t size, int w);

#endif /* _CV_ENCODING_JPEG_H */
