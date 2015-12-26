/*
 * Copyright (C) 2015 Kirk + Roland
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

/** @file modules/stereocam/stereoprotocol.h
 *  @brief standard protocol for TUDelft stereocamera data transfer
 */

#ifndef SW_AIRBORNE_MODULES_STEREO_CAM_STEREOPROTOCOL_H_
#define SW_AIRBORNE_MODULES_STEREO_CAM_STEREOPROTOCOL_H_

#include <inttypes.h>
#include "pprzlink/pprzlink_device.h"

struct MsgProperties {
  uint16_t positionImageStart;
  uint8_t width;
  uint8_t height;
} ;
typedef struct MsgProperties MsgProperties;
// function primitives

/**
 * Increment circular buffer counter by i
 */
uint16_t stereoprot_add(uint16_t counter, uint16_t i, uint16_t buffer_size);

/**
 * Decrement circular buffer counter by i
 */
uint16_t stereoprot_diff(uint16_t counter, uint16_t i, uint16_t buffer_size);

/**
 * Checks if the sequence in the array is equal to 255-0-0-171,
 * as this means that this is the end of an image
 */
uint8_t stereoprot_isEndOfMsg(uint8_t *stack, uint16_t i, uint16_t buffer_size);

/**
 * Checks if the sequence in the array is equal to 255-0-0-171,
 * as this means a new image is starting from here
 */
uint8_t stereoprot_isStartOfMsg(uint8_t *stack, uint16_t i, uint16_t buffer_size);
//void stereoprot_get_msg_properties(uint8_t *, MsgProperties *, uint16_t,uint16_t);

void WritePart(struct link_device *, uint8_t *, uint8_t);
void stereoprot_sendArray(struct link_device *fd, uint8_t *b, uint8_t array_width, uint8_t array_height);

/**
 * Get all available data from stereo com link and decode any complete messages.
 * Returns as soon as a complete message is found. Messages placed in msg_buf
 */
uint8_t handleStereoPackage(uint8_t newByte, uint16_t buffer_size, uint16_t *insert_loc, uint16_t *extract_loc,
                            uint16_t *msg_start, uint8_t *msg_buf, uint8_t *ser_read_buf, uint8_t *stereocam_datadata_new,
                            uint8_t *stereocam_datalen, uint8_t *stereocam_data_matrix_width, uint8_t *stereocam_data_matrix_height);

/**
 * Retrieve size of image from message
 */
void stereoprot_get_msg_properties(uint8_t *raw, MsgProperties *properties, uint16_t start, uint16_t buffer_size);




#endif /* SW_AIRBORNE_SUBSYSTEMS_GPS_STEREOPROTOCOL_H_ */
