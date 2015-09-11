/*
 * Copyright (C) 2015 Kirk Scheper
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

/** @file modules/sensors/stereocam.c
 *  @brief interface to TU Delft serial stereocam
 *  Include stereocam.xml to your airframe file.
 *  Parameters STEREO_PORT, STEREO_BAUD, SEND_STEREO and STEREO_BUF_SIZE should be configured with stereocam.xml.
 */

#include "stereocam.h"
#include "mcu_periph/uart.h"
#include "subsystems/datalink/telemetry.h"

#ifndef SEND_STEREO
#define SEND_STEREO TRUE
#endif

// define coms link for stereocam
#define STEREO_PORT   (&((UART_LINK).device))
struct link_device *dev = STEREO_PORT;

#define StereoGetch() STEREO_PORT ->get_byte(STEREO_PORT->periph)
#define StereoSend1(c) STEREO_PORT->put_byte(STEREO_PORT->periph, c)
#define StereoUartSend1(c) StereoSend1(c)
#define StereoSend(_dat,_len) { for (uint8_t i = 0; i< (_len); i++) StereoSend1(_dat[i]); };
#define StereoUartSetBaudrate(_b) uart_periph_set_baudrate(STEREO_PORT, _b);

typedef struct MsgProperties {
  uint16_t positionImageStart;
  uint8_t width;
  uint8_t height;
} MsgProperties;

// function primitives
uint16_t add(uint16_t, uint16_t);
uint16_t diff(uint16_t, uint16_t);
uint8_t isEndOfMsg(uint8_t *, uint16_t);
uint8_t isStartOfMsg(uint8_t *, uint16_t);
void get_msg_properties(uint8_t *, MsgProperties *, uint16_t);
static uint8_t handleStereoPackage(void);

// pervasive local variables
MsgProperties msgProperties;

uint8_t msg_buf[256];         // define local data
uint8array stereocam_data = {.len = 0, .data = msg_buf, .data_new = 0};  // buffer used to contain image without line endings
uint16_t freq_counter = 0;
uint16_t frequency = 0;
uint32_t previous_time = 0;

#ifndef STEREO_BUF_SIZE
#define STEREO_BUF_SIZE 1024                     // size of circular buffer
#endif
uint8_t ser_read_buf[STEREO_BUF_SIZE];           // circular buffer for incoming data
uint16_t insert_loc, extract_loc, msg_start;   // place holders for buffer read and write

/**
 * Increment circular buffer counter by i
 */
uint16_t add(uint16_t counter, uint16_t i)
{
  return (counter + i) % STEREO_BUF_SIZE;
}

/**
 * Decrement circular buffer counter by i
 */
uint16_t diff(uint16_t counter, uint16_t i)
{
  return (counter - i + STEREO_BUF_SIZE) % STEREO_BUF_SIZE;
}

/**
 * Checks if the sequence in the array is equal to 255-0-0-171,
 * as this means that this is the end of an image
 */
uint8_t isEndOfMsg(uint8_t *stack, uint16_t i)
{
  if (stack[i] == 255 && (stack[add(i, 1)] == 0) && (stack[add(i, 2)] == 0) && stack[add(i, 3)] == 171) {
    return 1;
  }
  return 0;
}

/**
 * Checks if the sequence in the array is equal to 255-0-0-171,
 * as this means a new image is starting from here
 */
uint8_t isStartOfMsg(uint8_t *stack, uint16_t i)
{
  if (stack[i] == 255 && (stack[add(i, 1)] == 0) && (stack[add(i, 2)] == 0) && stack[add(i, 3)] == 175) {
    return 1;
  }
  return 0;
}

/**
 * Retrieve size of image from message
 */
void get_msg_properties(uint8_t *raw, MsgProperties *properties, uint16_t start)
{
  *properties = (MsgProperties) {start, 0, 0};
  uint16_t i = start, startOfLine = start;
  while (1) {
    // Check the first 3 bytes for the pattern 255-0-0, then check what special byte is encoded next
    if ((raw[i] == 255) && (raw[add(i, 1)] == 0) && (raw[add(i, 2)] == 0)) {
      if (raw[add(i, 3)] == 171) { // End of image
        break;
      }
      if (raw[add(i, 3)] == 128) { // Start of line
        startOfLine = i;
      }
      if (raw[add(i, 3)] == 218) { // End of line
        properties->height++;
        properties->width = diff(i, startOfLine + 4); // removed 4 for the indication bits at the end of line
      }
    }
    i = add(i, 1);
  }
}

/**
 * Get all available data from stereo com link and decode any complete messages.
 * Returns as soon as a complete message is found. Messages placed in msg_buf
 */
static uint8_t handleStereoPackage(void)
{
  // read all data from the stereo com link, check that don't overtake extract
  while (dev->char_available(dev->periph) && add(insert_loc, 1) != extract_loc) {
    ser_read_buf[insert_loc] = dev->get_byte(dev->periph);
    insert_loc = add(insert_loc, 1);
  }

  // search for complete message in buffer, if found increments read location and returns immediately
  while (diff(insert_loc, extract_loc) > 0) {
    if (isStartOfMsg(ser_read_buf, extract_loc)) {
      msg_start = extract_loc;
    } else if (isEndOfMsg(ser_read_buf, extract_loc)) { // process msg
      // Find the properties of the image by iterating over the complete image
      get_msg_properties(ser_read_buf, &msgProperties, msg_start);

      // Copy array to circular buffer and remove all bytes that are indications of start and stop lines
      uint16_t i = add(msg_start, 8), j = 0, k = 0, index = 0;
      for (k = 0; k < msgProperties.height; k++) {
        for (j = 0; j < msgProperties.width; j++) {
          msg_buf[index++] = ser_read_buf[i];
          i = add(i, 1);
        }
        i = add(i, 8);    // step over EOL and SOL
      } // continue search for new line
      stereocam_data.len = msgProperties.width * msgProperties.height;
      stereocam_data.data_new = 1;
      extract_loc = add(extract_loc, 4);      // step over EOM string
      return 1;
    }
    extract_loc = add(extract_loc, 1);
  }
  return 0;
}

extern void stereocam_start(void)
{
  // initialize local variables
  msgProperties = (MsgProperties) {0, 0, 0};

  insert_loc = 0;
  extract_loc = 0;
  msg_start = 0;

  //sys_time_init();
  freq_counter = 0;
  frequency = 0;
  previous_time = sys_time.nb_tick;
}

extern void stereocam_stop(void)
{
}

extern void stereocam_periodic(void)
{
  if (handleStereoPackage()) {
    freq_counter++;
    if ((sys_time.nb_tick - previous_time) > sys_time.ticks_per_sec) {  // 1s has past
      frequency = (uint16_t)((freq_counter * (sys_time.nb_tick - previous_time)) / sys_time.ticks_per_sec);
      freq_counter = 0;
      previous_time = sys_time.nb_tick;
    }
#if SEND_STEREO
    DOWNLINK_SEND_STEREO_IMG(DefaultChannel, DefaultDevice, &frequency, &(stereocam_data.len), stereocam_data.len, msg_buf);
#endif
  }
}
