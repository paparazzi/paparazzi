/*
 * Copyright (C) 2015
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
 * @file modules/computer_vision/qrcode/qr_code.h
 *
 * Parse video stream to detect and decode QR-codes using the ZBAR library
 */

#ifndef QR_CODE_MODULE
#define QR_CODE_MODULE


#include <stdint.h>

#include "../lib/vision/image.h"

extern void qrcode_init(void);
extern bool_t qrscan(struct image_t *img);


#endif
