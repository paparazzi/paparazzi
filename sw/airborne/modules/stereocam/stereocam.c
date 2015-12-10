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

/** @file modules/stereocam/stereocam.c
 *  @brief interface to TU Delft serial stereocam
 *  Include stereocam.xml to your airframe file.
 *  Parameters STEREO_PORT, STEREO_BAUD, SEND_STEREO and STEREO_BUF_SIZE should be configured with stereocam.xml.
 */

#include "modules/stereocam/stereocam.h"
#include "mcu_periph/uart.h"
#include "subsystems/datalink/telemetry.h"
#include "modules/stereocam/stereoprotocol.h"
#ifndef SEND_STEREO
#define SEND_STEREO TRUE
#endif

// define coms link for stereocam
#define STEREO_PORT   (&((UART_LINK).device))
struct link_device *linkdev = STEREO_PORT;
#define StereoGetch() STEREO_PORT ->get_byte(STEREO_PORT->periph)

// pervasive local variables
MsgProperties msgProperties;


uint16_t freq_counter = 0;
uint8_t frequency = 0;
uint32_t previous_time = 0;

#ifndef STEREO_BUF_SIZE
#define STEREO_BUF_SIZE 1024                     // size of circular buffer
#endif
uint8_t ser_read_buf[STEREO_BUF_SIZE];           // circular buffer for incoming data
uint16_t insert_loc, extract_loc, msg_start;   // place holders for buffer read and write
uint8_t msg_buf[STEREO_BUF_SIZE];         // define local data
uint8array stereocam_data = {.len = 0, .data = msg_buf, .fresh = 0, .matrix_width = 0, .matrix_height = 0}; // buffer used to contain image without line endings

#define BASELINE_STEREO_MM 60.0
#define BRANDSPUNTSAFSTAND_STEREO 118.0*6

extern void stereocam_disparity_to_meters(uint8_t *disparity, float *distancesMeters, int lengthArray)
{

  int indexArray = 0;
  for (indexArray = 0; indexArray < lengthArray; indexArray++) {
    if (disparity[indexArray] != 0) {
      distancesMeters[indexArray] = ((BASELINE_STEREO_MM * BRANDSPUNTSAFSTAND_STEREO / (float)disparity[indexArray] - 18.0)) /
                                    1000;
      //  printf("%i, distanceMeters: %f \n",indexArray,distancesMeters[indexArray]);
    } else {
      distancesMeters[indexArray] = 1000;
    }
  }
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

  stereocam_data.fresh = 0;
}

extern void stereocam_stop(void)
{
}

extern void stereocam_periodic(void)
{
  // read all data from the stereo com link, check that don't overtake extract
  while (linkdev->char_available(linkdev->periph) && stereoprot_add(insert_loc, 1, STEREO_BUF_SIZE) != extract_loc) {
    if (handleStereoPackage(StereoGetch(), STEREO_BUF_SIZE, &insert_loc, &extract_loc, &msg_start, msg_buf, ser_read_buf,
                            &stereocam_data.fresh, &stereocam_data.len, &stereocam_data.matrix_width, &stereocam_data.matrix_height)) {
      freq_counter++;
      if ((sys_time.nb_tick - previous_time) > sys_time.ticks_per_sec) {  // 1s has past
        frequency = (uint8_t)((freq_counter * (sys_time.nb_tick - previous_time)) / sys_time.ticks_per_sec);
        freq_counter = 0;
        previous_time = sys_time.nb_tick;
      }
#if SEND_STEREO
      if (stereocam_data.len > 100) {
        DOWNLINK_SEND_STEREO_IMG(DefaultChannel, DefaultDevice, &frequency, &(stereocam_data.len), 100, stereocam_data.data);

      } else {
        DOWNLINK_SEND_STEREO_IMG(DefaultChannel, DefaultDevice, &frequency, &(stereocam_data.len), stereocam_data.len,
                                 stereocam_data.data);

      }
#endif
    }
  }
}
