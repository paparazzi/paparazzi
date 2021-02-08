/*
 * Copyright (C) 2013 Martin Mueller <martinmm@pfump.org>
 * Copyright (C) 2016 Gautier Hattenberger <gautier.hattenberger@enac.fr>
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
 * @file modules/sensors/airspeed_uADC.c
 * UART interface for Aeroprobe uADC air data computer.
 *
 */

#include "airspeed_uADC.h"

#include "mcu_periph/uart.h"
#include "pprzlink/messages.h"
#include "subsystems/datalink/downlink.h"

#include <ctype.h>
#include <stdlib.h>

#if FLIGHTRECORDER_SDLOG
#include "subsystems/datalink/telemetry.h"
#include "modules/loggers/pprzlog_tp.h"
#include "modules/loggers/sdlog_chibios.h"
#endif

#define uADC_UNINIT         0
#define uADC_WAIT_START     uADC_UNINIT
#define uADC_WAIT_COUNTER   1
#define uADC_WAIT_ANGLES    2
#define uADC_WAIT_ALTITUDE  3
#define uADC_WAIT_PRESSURES 4
#define uADC_WAIT_CHECKSUM  5

#define uADC_START   0x0A
#define uADC_LIMITER ','
#define uADC_END     0x0D

static uint32_t counter;
static int16_t course[3];
static int32_t altitude;
static int32_t pressures[2];
static uint8_t checksum;
static uint8_t uadc_status = uADC_UNINIT;
static uint8_t uadc_idx = 0;
static uint8_t uadc_tab_idx = 0;

/* airspeed_uadc_parse */
void airspeed_uadc_parse(char c)
{
  static char uadc_inp[64];

  switch (uadc_status) {

    case uADC_WAIT_START:
      if (c == uADC_START) {
        uadc_status++;
        uadc_idx = 0;
      } else {
        uadc_status = uADC_UNINIT;
      }
      break;

    case uADC_WAIT_COUNTER:
      if (isdigit((int)c)) {
        uadc_inp[uadc_idx++] = c;
      } else {
        if (/*(uadc_idx == 5) &&*/ (c == uADC_LIMITER)) {
          uadc_inp[uadc_idx] = 0;
          counter = atoi(uadc_inp);
          uadc_idx = 0;
          uadc_tab_idx = 0;
          uadc_status++;
        } else {
          uadc_status = uADC_UNINIT;
        }
      }
      break;

    case uADC_WAIT_ANGLES:
      if (isdigit((int)c) || (c == '-') || (c == '.')) {
        uadc_inp[uadc_idx++] = c;
      } else {
        if ((uadc_idx > 1) && (uadc_idx < 9) && (c == uADC_LIMITER)) {
          uadc_inp[uadc_idx] = 0;
          course[uadc_tab_idx] = (int16_t)(100. * atof(uadc_inp));
          uadc_idx = 0;
          if (uadc_tab_idx++ == 2) {
            uadc_tab_idx = 0;
            uadc_status++;
          }
        } else if ((c == ' ') || (c == '+')) {
          // skip blank and + characters
        }
        else {
          // illigal character, reset parser
          uadc_status = uADC_UNINIT;
        }
      }
      break;

    case uADC_WAIT_ALTITUDE:
      if (isdigit((int)c) || (c == '-') || (c == '.')) {
        uadc_inp[uadc_idx++] = c;
      } else {
        if ((uadc_idx > 1) && (uadc_idx < 9) && (c == uADC_LIMITER)) {
          uadc_inp[uadc_idx] = 0;
          altitude = (int32_t)(100. * atof(uadc_inp));
          uadc_idx = 0;
          uadc_status++;
        } else if ((c == ' ') || (c == '+')) {
          // skip blank and + characters
        }
        else {
          // illigal character, reset parser
          uadc_status = uADC_UNINIT;
        }
      }
      break;

    case uADC_WAIT_PRESSURES:
      if (isdigit((int)c) || (c == '-') || (c == '.')) {
        uadc_inp[uadc_idx++] = c;
      } else {
        if ((uadc_idx > 1) && (uadc_idx < 9) && (c == uADC_LIMITER)) {
          uadc_inp[uadc_idx] = 0;
          pressures[uadc_tab_idx] = (int32_t) atoi(uadc_inp);
          uadc_idx = 0;
          if (uadc_tab_idx++ == 1) {
            uadc_tab_idx = 0;
            uadc_status++;
          }
        } else if ((c == ' ') || (c == '+')) {
          // skip blank and + characters
        }
        else {
          // illigal character, reset parser
          uadc_status = uADC_UNINIT;
        }
      }
      break;

    case uADC_WAIT_CHECKSUM:
      if (isxdigit((int)c)) {
        uadc_inp[uadc_idx++] = c;
      } else {
        if ((uadc_idx == 2) && (c == uADC_END)) {
          uadc_inp[uadc_idx] = 0;
          checksum = strtol(uadc_inp, NULL, 16);
          uadc_idx = 0;
          // FIXME update message definition
#if FLIGHTRECORDER_SDLOG
          if (flightRecorderLogFile != -1) {
            DOWNLINK_SEND_AEROPROBE(pprzlog_tp, flightrecorder_sdlog,
                &counter, &course[0], &course[1], &course[2],
                &altitude, &pressures[0], &pressures[1], &checksum);
          }
#endif
          uadc_status = uADC_UNINIT;
        } else if ((c == ' ')) {
          // skip blank characters
        }
        else {
          uadc_status = uADC_UNINIT;
        }
      }
      break;

    default:
      uadc_status = uADC_UNINIT;
      break;
  }
}

void airspeed_uADC_init(void)
{
}

void airspeed_uADC_event(void)
{
  while (uart_char_available(&(uADC_DEV))) {
    uint8_t ch = uart_getch(&(uADC_DEV));
    airspeed_uadc_parse(ch);
  }
}

void airspeed_uADC_periodic(void)
{
  DOWNLINK_SEND_AEROPROBE(DefaultChannel, DefaultDevice,
      &counter, &course[0], &course[1], &course[2],
      &altitude, &pressures[0], &pressures[1], &checksum);
}

