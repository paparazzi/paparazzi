/*
 * Copyright (C) 2013 Martin Mueller <martinmm@pfump.org>
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

/**
 * @file modules/sensors/airspeed_otf.c
 * UART interface for Aeroprobe On-The-Fly! air data computer.
 *
 */

#include <stdbool.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include "mcu_periph/uart.h"
#include "messages.h"
#include "subsystems/datalink/downlink.h"

#include "met_module.h"
#include "airspeed_otf.h"


#define OTF_UNINIT         0x00
#define OTF_WAIT_START     OTF_UNINIT
#define OTF_WAIT_COUNTER   0x01
#define OTF_WAIT_ANGLES    0x02
#define OTF_WAIT_ALTITUDE  0x03
#define OTF_WAIT_CHECKSUM  0x04

#define OTF_START   0x0A
#define OTF_LIMITER ','
#define OTF_END     0x0D

/* workaround for newlib */
void *_sbrk(int);
void *_sbrk(int a) {return 0;}

/* airspeed_otf_parse */
void airspeed_otf_parse(char c)
{
  static unsigned char otf_status = OTF_UNINIT, otf_idx = 0, otf_crs_idx;
  static char otf_inp[64];
  static unsigned int counter;
  static short course[3];
  static unsigned int altitude;
  static unsigned char checksum;

  switch (otf_status) {

    case OTF_WAIT_START:
      if (c == OTF_START) {
        otf_status++;
        otf_idx = 0;
      } else {
        otf_status = OTF_UNINIT;
      }
      break;

    case OTF_WAIT_COUNTER:
      if (isdigit((int)c)) {
        if (otf_idx == 0) {
//FIXME        otf_timestamp = getclock();
        }
        otf_inp[otf_idx++] = c;
      } else {
        if ((otf_idx == 5) && (c == OTF_LIMITER)) {
          otf_inp[otf_idx] = 0;
          counter = atoi(otf_inp);
          otf_idx = 0;
          otf_crs_idx = 0;
          otf_status++;
        } else {
          otf_status = OTF_UNINIT;
        }
      }
      break;

    case OTF_WAIT_ANGLES:
      if (isdigit((int)c) || (c == '-') || (c == '.')) {
        otf_inp[otf_idx++] = c;
      } else {
        if ((otf_idx > 1) && (otf_idx < 9) && (c == OTF_LIMITER)) {
          otf_inp[otf_idx] = 0;
          course[otf_crs_idx] = (int16_t)(100. * atof(otf_inp));
          otf_idx = 0;
          if (otf_crs_idx++ == 2) {
            otf_status++;
          }
        } else {
          otf_status = OTF_UNINIT;
        }
      }
      break;

    case OTF_WAIT_ALTITUDE:
      if (isdigit((int)c) || (c == '-') || (c == '.')) {
        otf_inp[otf_idx++] = c;
      } else {
        if ((otf_idx > 1) && (otf_idx < 9) && (c == OTF_LIMITER)) {
          otf_inp[otf_idx] = 0;
          altitude = (int32_t)(100. * atof(otf_inp));
          otf_idx = 0;
          otf_status++;
        } else {
          otf_status = OTF_UNINIT;
        }
      }
      break;

    case OTF_WAIT_CHECKSUM:
      if (isxdigit((int)c)) {
        otf_inp[otf_idx++] = c;
      } else {
        if ((otf_idx == 2) && (c == OTF_END)) {
          otf_inp[otf_idx] = 0;
          checksum = strtol(otf_inp, NULL, 16);
          otf_idx = 0;
          DOWNLINK_SEND_FLOW_AP_OTF(DefaultChannel, DefaultDevice, &counter, &course[0], &course[1], &course[2], &altitude,
                                    &checksum);
        }
        otf_status = OTF_UNINIT;
      }
      break;

    default:
      otf_status = OTF_UNINIT;
      break;
  }
}

void airspeed_otf_init(void)
{
}

void airspeed_otf_event(void)
{
  while (MetBuffer()) {
    uint8_t ch = MetGetch();
    airspeed_otf_parse(ch);
  }
}

void airspeed_otf_periodic(void)
{
}
