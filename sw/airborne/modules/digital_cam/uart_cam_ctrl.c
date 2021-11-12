/*
 * Copyright (C) OpenUAS
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
 */

/**
 * @file modules/digital_cam/uart_cam_ctrl.c
 * Control the camera via uart to chdk-ptp.
 * Retrieve thumbnails
 */

#include "uart_cam_ctrl.h"
#include "generated/airframe.h"

// Include Standard Camera Control Interface
#include "modules/digital_cam/dc.h"

// Telemetry
#include "modules/datalink/telemetry.h"

#include BOARD_CONFIG

// Communication
#include "modules/digital_cam/catia/protocol.h"

#ifdef SITL
#include "modules/digital_cam/catia/serial.h"
#endif

#include "state.h"


#define CameraLinkDev (&((CAMERA_LINK).device))
#define CameraLinkTransmit(c) CameraLinkDev->put_byte(CameraLinkDev->periph, 0, c)
#define CameraLinkChAvailable() CameraLinkDev->char_available(CameraLinkDev->periph)
#define CameraLinkGetch() CameraLinkDev->get_byte(CameraLinkDev->periph)

union dc_shot_union dc_shot_msg;
union mora_status_union mora_status_msg;
int digital_cam_uart_status = 0;

int digital_cam_uart_thumbnails = 0;
#define THUMB_MSG_SIZE  MORA_PAYLOAD_MSG_SIZE
#define THUMB_COUNT     10
static uint8_t thumbs[THUMB_COUNT][THUMB_MSG_SIZE];
static uint8_t thumb_pointer = 0;


void digital_cam_uart_event(void)
{
  while (CameraLinkChAvailable()) {
    parse_mora(&mora_protocol, CameraLinkGetch());
    if (mora_protocol.msg_received) {
      switch (mora_protocol.msg_id) {
        case MORA_STATUS:
          for (int i = 0; i < MORA_STATUS_MSG_SIZE; i++) {
            mora_status_msg.bin[i] = mora_protocol.payload[i];
          }
          digital_cam_uart_status = mora_status_msg.data.shots;
          break;
        case MORA_PAYLOAD:
          for (int i = 0; i < MORA_PAYLOAD_MSG_SIZE; i++) {
            thumbs[thumb_pointer][i] = mora_protocol.payload[i];
          }
          break;
        default:
          break;
      }
      mora_protocol.msg_received = 0;
    }
  }
}

#if PERIODIC_TELEMETRY
static void send_thumbnails(struct transport_tx *trans, struct link_device *dev)
{
  static int cnt = 0;
  if (digital_cam_uart_thumbnails > 0) {
    if (digital_cam_uart_thumbnails == 1) {
      cnt++;
      if (cnt > 1) {
        cnt = 0;
        return;
      }
    }
    pprz_msg_send_PAYLOAD(trans, dev, AC_ID, THUMB_MSG_SIZE, thumbs[thumb_pointer]);

    // Update the write/read pointer: if we receive a new thumb part, that will be sent, otherwise the oldest infor is repeated
    thumb_pointer++;
    if (thumb_pointer >= THUMB_COUNT) {
      thumb_pointer = 0;
    }

    MoraHeader(MORA_BUFFER_EMPTY, 0);
    MoraTrailer();
  }
}
#endif

void digital_cam_uart_init(void)
{
  // Call common DC init
  dc_init();
  digital_cam_uart_thumbnails = 0;
  for (int t = 0; t < THUMB_COUNT; t++) {
    for (int i = 0; i < THUMB_MSG_SIZE; i++) {
      thumbs[t][i] = 0;
    }
  }
#if PERIODIC_TELEMETRY
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_PAYLOAD, send_thumbnails);
#endif

#ifdef SITL
  serial_init("/dev/ttyUSB0");
#endif
}

void digital_cam_uart_periodic(void)
{
  // Common DC Periodic task
  dc_periodic();
}


/* Command The Camera */
void dc_send_command(uint8_t cmd)
{
  switch (cmd) {
    case DC_SHOOT:
      // Send Photo Position To Camera
      dc_shot_msg.data.nr = dc_photo_nr + 1;
      dc_shot_msg.data.lat = stateGetPositionLla_i()->lat;
      dc_shot_msg.data.lon = stateGetPositionLla_i()->lon;
      dc_shot_msg.data.alt = stateGetPositionLla_i()->alt;
      dc_shot_msg.data.phi = stateGetNedToBodyEulers_i()->phi;
      dc_shot_msg.data.theta = stateGetNedToBodyEulers_i()->theta;
      dc_shot_msg.data.psi = stateGetNedToBodyEulers_i()->psi;
      dc_shot_msg.data.vground = stateGetHorizontalSpeedNorm_i();
      dc_shot_msg.data.course = stateGetHorizontalSpeedDir_i();
      dc_shot_msg.data.groundalt = POS_BFP_OF_REAL(state.alt_agl_f);

      MoraHeader(MORA_SHOOT, MORA_SHOOT_MSG_SIZE);
      for (int i = 0; i < (MORA_SHOOT_MSG_SIZE); i++) {
        MoraPutUint8(dc_shot_msg.bin[i]);
      }
      MoraTrailer();
      dc_send_shot_position();
      break;
    case DC_TALLER:
      break;
    case DC_WIDER:
      break;
    case DC_ON:
      break;
    case DC_OFF:
      break;
    default:
      break;
  }

  // call command send_command function
  dc_send_command_common(cmd);
}
