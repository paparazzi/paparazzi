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
 * @file modules/computer_vision/qrcode/qr_code.c
 */

#include "qr_code.h"
#include "cv.h"

#include "zbar.h"
#include <stdio.h>

void qrcode_init(void)
{
	// Add qrscan to the list of image processing tasks in video_thread
  cv_add(qrscan);
}

// Telemetry
#include "subsystems/datalink/telemetry.h"


zbar_image_scanner_t *scanner = 0;

bool_t qrscan(struct image_t *img)
{
  int i, j;

  // Create the JPEG encoded image
  struct image_t gray;
  image_create(&gray, img->w, img->h, IMAGE_GRAYSCALE);

  uint8_t *ii = (uint8_t *) img->buf;
  uint8_t *oi = (uint8_t *) gray.buf;

  for (j = 0; j < img->h; j++) {
    for (i = 0; i < img->w; i++) {
      oi[j * img->w + i] = ii[(j * img->w + i) * 2 + 1];
    }
  }

  if (scanner == 0) {
    // create a reader
    scanner = zbar_image_scanner_create();

    // configure the reader
    //zbar_image_scanner_set_config(scanner, 0, ZBAR_CFG_POSITION, 1);
    //zbar_increase_verbosity();
  }

  // wrap image data
  zbar_image_t *image = zbar_image_create();
  zbar_image_set_format(image, *(int *)"Y800");
  zbar_image_set_size(image, gray.w, gray.h);
  zbar_image_set_data(image, gray.buf, gray.buf_size, zbar_image_free_data);

  // scan the image for barcodes
  int n = zbar_scan_image(scanner, image);

  if (n < 0) {
    printf("zbar_scan_image returned %d\n", n);
  }

  // extract results
  const zbar_symbol_t *symbol = zbar_image_first_symbol(image);
  for (; symbol; symbol = zbar_symbol_next(symbol)) {
    // do something useful with results
    zbar_symbol_type_t typ = zbar_symbol_get_type(symbol);
    char *data = (char *)zbar_symbol_get_data(symbol);
    printf("decoded %s symbol \"%s\"\n",
           zbar_get_symbol_name(typ), data);

    // TODO: not allowed to access telemetry from vision thread
#if DOWNLINK
    DOWNLINK_SEND_INFO_MSG(DefaultChannel, DefaultDevice, strlen(data), data);
#endif
  }

// clean up
  zbar_image_destroy(image);
  //zbar_image_scanner_destroy(scanner);
}
