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

/** @file modules/stereocam/stereoprotocol.c
 *  @brief standard protocol for TUDelft stereocamera data transfer
 */

#include "stereoprotocol.h"

/**
 * Increment circular buffer counter by i
 */
uint16_t stereoprot_add(uint16_t counter, uint16_t i, uint16_t buffer_size)
{
  return (counter + i) % buffer_size;
}


/**
 * Decrement circular buffer counter by i
 */
uint16_t stereoprot_diff(uint16_t counter, uint16_t i, uint16_t buffer_size)
{
  return (counter - i + buffer_size) % buffer_size;
}

/**
 * Checks if the sequence in the array is equal to 255-0-0-171,
 * as this means that this is the end of an image
 */
uint8_t stereoprot_isEndOfMsg(uint8_t *stack, uint16_t i, uint16_t buffer_size)
{

  if (stack[i] == 255 && (stack[stereoprot_add(i, 1, buffer_size)] == 0)
      && (stack[stereoprot_add(i, 2, buffer_size)] == 0) && stack[stereoprot_add(i, 3, buffer_size)] == 171) {
    return 1;
  }
  return 0;
}

/**
 * Checks if the sequence in the array is equal to 255-0-0-171,
 * as this means a new image is starting from here
 */
uint8_t stereoprot_isStartOfMsg(uint8_t *stack, uint16_t i, uint16_t buffer_size)
{
  //printf("Checking start: %d %d %d %d \n",stack[i],stack[stereoprot_add(i, 1,buffer_size)],stack[stereoprot_add(i, 2,buffer_size)],stack[stereoprot_add(i, 3,buffer_size)]);
  if (stack[i] == 255 && (stack[stereoprot_add(i, 1, buffer_size)] == 0)
      && (stack[stereoprot_add(i, 2, buffer_size)] == 0) && stack[stereoprot_add(i, 3, buffer_size)] == 175) {
    return 1;
  }
  return 0;
}

//void stereoprot_get_msg_properties(uint8_t *, MsgProperties *, uint16_t,uint16_t);


void WritePart(struct link_device *dev, uint8_t *code, uint8_t length)
{
  long fd = 0;
  if (dev->check_free_space(dev->periph, &fd, length)) {
    for (uint8_t index = 0; index < length; index++) {
      dev->put_byte(dev->periph, fd, code[index]);
    }
  }
}
void stereoprot_sendArray(struct link_device *fd, uint8_t *b, uint8_t array_width, uint8_t array_height)
{

  uint8_t code[4];
  code[0] = 0xff;
  code[1] = 0x00;
  code[2] = 0x00;
  code[3] = 0xAF; // 175
  WritePart(fd, code, 4);


  int horizontalLine = 0;
  for (horizontalLine = 0; horizontalLine < array_height; horizontalLine++) {
    code[3] = 0x80;//128
    WritePart(fd, code, 4);
    WritePart(fd, b + array_width * horizontalLine, array_width);

    code[3] = 0xDA;//218
    WritePart(fd, code, 4);
  }

  code[3] = 0xAB;
  WritePart(fd, code, 4);
}

/**
 * Get all available data from stereo com link and decode any complete messages.
 * Returns as soon as a complete message is found. Messages placed in msg_buf
 */
uint8_t handleStereoPackage(uint8_t newByte, uint16_t buffer_size, uint16_t *insert_loc, uint16_t *extract_loc,
                            uint16_t *msg_start, uint8_t *msg_buf, uint8_t *ser_read_buf, uint8_t *stereocam_datadata_new,
                            uint8_t *stereocam_datalen, uint8_t *stereocam_data_matrix_width, uint8_t *stereocam_data_matrix_height)
{


  MsgProperties msgProperties;
  // read all data from the stereo com link, check that don't overtake extract
  if (stereoprot_add(*insert_loc, 1, buffer_size) != *extract_loc) {
    ser_read_buf[*insert_loc] = newByte;
    *insert_loc = stereoprot_add(*insert_loc, 1, buffer_size);
  }


  // search for complete message in buffer, if found increments read location and returns immediately

  //while (stereoprot_diff(*insert_loc, stereoprot_add(*extract_loc,3,buffer_size),buffer_size) > 0) {
  while (stereoprot_diff(*insert_loc, *extract_loc, buffer_size) > 3) {
    if (stereoprot_isStartOfMsg(ser_read_buf, *extract_loc, buffer_size)) {


      *msg_start = *extract_loc;
    } else if (stereoprot_isEndOfMsg(ser_read_buf, *extract_loc, buffer_size)) { // process msg


      // Find the properties of the image by iterating over the complete image
      stereoprot_get_msg_properties(ser_read_buf, &msgProperties, *msg_start, buffer_size);
      // Copy array to circular buffer and remove all bytes that are indications of start and stop lines
      uint16_t i = stereoprot_add(*msg_start, 8, buffer_size), j = 0, k = 0, index = 0;
      for (k = 0; k < msgProperties.height; k++) {
        for (j = 0; j < msgProperties.width; j++) {
          msg_buf[index++] = ser_read_buf[i];
          i = stereoprot_add(i, 1, buffer_size);
        }
        i = stereoprot_add(i, 8, buffer_size);   // step over EOL and SOL
      } // continue search for new line
      *stereocam_datalen = msgProperties.width * msgProperties.height;
      *stereocam_data_matrix_width = msgProperties.width;
      *stereocam_data_matrix_height = msgProperties.height;

      *stereocam_datadata_new = 1;
      *extract_loc = stereoprot_add(*extract_loc, 4, buffer_size);     // step over EOM string

      return 1;
    }
    *extract_loc = stereoprot_add(*extract_loc, 1, buffer_size);

  }
  return 0;
}


/**
 * Retrieve size of image from message
 */
void stereoprot_get_msg_properties(uint8_t *raw, MsgProperties *properties, uint16_t start, uint16_t buffer_size)
{
  *properties = (MsgProperties) {start, 0, 0};
  uint16_t i = start, startOfLine = start;
  while (1) {
    // Check the first 3 bytes for the pattern 255-0-0, then check what special byte is encoded next
    if ((raw[i] == 255) && (raw[stereoprot_add(i, 1, buffer_size)] == 0) && (raw[stereoprot_add(i, 2, buffer_size)] == 0)) {
      if (raw[stereoprot_add(i, 3, buffer_size)] == 171) { // End of image
        break;
      }
      if (raw[stereoprot_add(i, 3, buffer_size)] == 128) { // Start of line
        startOfLine = i;
      }
      if (raw[stereoprot_add(i, 3, buffer_size)] == 218) { // End of line
        properties->height++;
        properties->width = stereoprot_diff(i, startOfLine + 4,
                                            buffer_size); // removed 4 for the indication bits at the end of line
      }
    }
    i = stereoprot_add(i, 1, buffer_size);
  }
}
