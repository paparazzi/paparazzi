/*
 * Copyright (C) 2013 Sergey Krukowski <softsr@yahoo.de>
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

/** @file hott.c
 *
 * Graupner HOTT telemetry control module.
 */

#include "hott/hott.h"
#if HOTT_SIM_GAM_SENSOR
#include "hott/hott_gam.h"
#endif
#if HOTT_SIM_EAM_SENSOR
#include "hott/hott_eam.h"
#endif
#if HOTT_SIM_GPS_SENSOR
#include "hott/hott_gps.h"
#endif
#if HOTT_SIM_VARIO_SENSOR
#include "hott/hott_vario.h"
#endif

#include "mcu_periph/sys_time.h"
#include "mcu_periph/uart.h"

#define HOTT_TEXT_MODE_REQUEST_ID  0x7f
#define HOTT_BINARY_MODE_REQUEST_ID  0x80
//Sensor Ids

//Graupner #33600 GPS Module
#define HOTT_TELEMETRY_GPS_SENSOR_ID  0x8A
//Graupner #33601 Vario Module
#define HOTT_TELEMETRY_VARIO_SENSOR_ID  0x89

static uint32_t hott_event_timer; // 1ms software timer
static bool hott_telemetry_is_sending = false;
static uint16_t hott_telemetry_sendig_msgs_id = 0;

#if HOTT_SIM_GPS_SENSOR
bool  HOTT_REQ_UPDATE_GPS = false;
#endif
#if HOTT_SIM_EAM_SENSOR
bool  HOTT_REQ_UPDATE_EAM = false;
#endif
#if HOTT_SIM_VARIO_SENSOR
bool  HOTT_REQ_UPDATE_VARIO = false;
#endif
#if HOTT_SIM_GAM_SENSOR
bool  HOTT_REQ_UPDATE_GAM  = false;
#endif

// HoTT serial send buffer pointer
static int8_t *hott_msg_ptr = 0;
// Len of HoTT serial buffer
static int16_t hott_msg_len = 0;

#if HOTT_SIM_GAM_SENSOR
static struct HOTT_GAM_MSG hott_gam_msg;
#endif
#if HOTT_SIM_EAM_SENSOR
static struct HOTT_EAM_MSG hott_eam_msg;
#endif
#if HOTT_SIM_GPS_SENSOR
static struct HOTT_GPS_MSG hott_gps_msg;
#endif
#if HOTT_SIM_VARIO_SENSOR
static struct HOTT_VARIO_MSG hott_vario_msg;
#endif

/**
 * Initializes a HoTT GPS message (Receiver answer type  !Not Smartbox)
 */
static void hott_msgs_init(void)
{

#if HOTT_SIM_EAM_SENSOR
  hott_init_eam_msg(&hott_eam_msg);
#endif
#if HOTT_SIM_GAM_SENSOR
  hott_init_gam_msg(&hott_gam_msg);
#endif
#if HOTT_SIM_GPS_SENSOR
  hott_init_gps_msg(&hott_gam_msg);
#endif
#if HOTT_SIM_VARIO_SENSOR
  hott_init_vario_msg(&hott_gam_msg);
#endif
#if HOTT_SIM_TEXTMODE
  hott_init_text_msg(&hott_gam_msg);
#endif
}

static void hott_enable_transmitter(void)
{
  //enables serial transmitter, disables receiver
  uart_periph_set_mode(&HOTT_PORT, TRUE, FALSE, FALSE);
}

static void hott_enable_receiver(void)
{
  //enables serial receiver, disables transmitter
  uart_periph_set_mode(&HOTT_PORT, TRUE, TRUE, FALSE);
}

void hott_init(void)
{
  hott_msgs_init();
}

/**/
void hott_periodic(void)
{
#if HOTT_SIM_EAM_SENSOR
  if ((hott_telemetry_sendig_msgs_id != HOTT_TELEMETRY_EAM_SENSOR_ID) &&
      HOTT_REQ_UPDATE_EAM == TRUE) {
    hott_update_eam_msg(&hott_eam_msg);
    HOTT_REQ_UPDATE_EAM = false;
  }
#endif
#if HOTT_SIM_GAM_SENSOR
  if ((hott_telemetry_sendig_msgs_id != HOTT_TELEMETRY_GAM_SENSOR_ID) &&
      HOTT_REQ_UPDATE_GAM == TRUE) {
    hott_update_gam_msg(&hott_gam_msg);
    HOTT_REQ_UPDATE_GAM = false;
  }
#endif
#if HOTT_SIM_GPS_SENSOR
  if ((hott_telemetry_sendig_msgs_id != HOTT_TELEMETRY_GPS_SENSOR_ID) &&
      HOTT_REQ_UPDATE_GPS == TRUE) {
    hott_update_gps_msg(&hott_gam_msg);
    HOTT_REQ_UPDATE_GPS = false;
  }
#endif
#if HOTT_SIM_VARIO_SENSOR
  if ((hott_telemetry_sendig_msgs_id != HOTT_TELEMETRY_VARIO_SENSOR_ID) &&
      HOTT_REQ_UPDATE_VARIO == TRUE) {
    hott_update_vario_msg(&hott_gam_msg);
    HOTT_REQ_UPDATE_VARIO = false;
  }
#endif
}

static void hott_send_msg(int8_t *buffer, int16_t len)
{
  if (hott_telemetry_is_sending == TRUE) { return; }
  hott_msg_ptr = buffer;
  hott_msg_len = len + 1; //len + 1 byte for crc
  hott_telemetry_sendig_msgs_id = buffer[1];  //HoTT msgs id is the 2. byte
}

static void hott_send_telemetry_data(void)
{
  static int16_t msg_crc = 0;
  if (!hott_telemetry_is_sending) {
    hott_telemetry_is_sending = true;
    hott_enable_transmitter();
  }

  if (hott_msg_len == 0) {
    hott_msg_ptr = 0;
    hott_telemetry_is_sending = false;
    hott_telemetry_sendig_msgs_id = 0;
    msg_crc = 0;
    hott_enable_receiver();
    while (uart_char_available(&HOTT_PORT)) {
      uart_getch(&HOTT_PORT);
    }
  } else {
    --hott_msg_len;
    if (hott_msg_len != 0) {
      msg_crc += *hott_msg_ptr;
      uart_put_byte(&HOTT_PORT, 0, *hott_msg_ptr++);
    } else {
      uart_put_byte(&HOTT_PORT, 0, (int8_t)msg_crc);
    }
  }
}

static void hott_check_serial_data(uint32_t tnow)
{
  static uint32_t hott_serial_request_timer = 0;
  if (hott_telemetry_is_sending == TRUE) { return; }
  if (uart_char_available(&HOTT_PORT) > 1) {
    if (uart_char_available(&HOTT_PORT) == 2) {
      if (hott_serial_request_timer == 0) {
        hott_serial_request_timer = tnow;
        return;
      } else {
        if (tnow - hott_serial_request_timer < 4600) { //wait ca. 5ms
          return;
        }
        hott_serial_request_timer = 0;
      }
      uint8_t c = uart_getch(&HOTT_PORT);
      uint8_t addr = uart_getch(&HOTT_PORT);

      switch (c) {
#if HOTT_SIM_TEXTMODE
        case HOTT_TEXT_MODE_REQUEST_ID:
          //Text mode, handle only if not armed!
          if (!autopilot_motors_on) {
            hott_txt_msg.start_byte = 0x7b;
            hott_txt_msg.stop_byte = 0x7d;
            uint8_t tmp = (addr >> 4);  // Sensor type
            if (tmp == (HOTT_SIM_TEXTMODE_ADDRESS & 0x0f)) {
              HOTT_Clear_Text_Screen();
              HOTT_HandleTextMode(addr);
              hott_send_text_msg();
            }
          }
          break;
#endif
        case HOTT_BINARY_MODE_REQUEST_ID:
#if HOTT_SIM_EAM_SENSOR
          if (addr == HOTT_TELEMETRY_EAM_SENSOR_ID) {
            hott_send_msg((int8_t *)&hott_eam_msg, sizeof(struct HOTT_EAM_MSG));
            HOTT_REQ_UPDATE_EAM = true;
            break;
          }
#endif
#if HOTT_SIM_GAM_SENSOR
          if (addr == HOTT_TELEMETRY_GAM_SENSOR_ID) {
            hott_send_msg((int8_t *)&hott_gam_msg, sizeof(struct HOTT_GAM_MSG));
            HOTT_REQ_UPDATE_GAM = true;
            break;
          }
#endif
#if HOTT_SIM_GPS_SENSOR
          if (addr == HOTT_TELEMETRY_GPS_SENSOR_ID) {
            hott_send_msg((int8_t *)&hott_gps_msg, sizeof(struct HOTT_GPS_MSG));
            HOTT_REQ_UPDATE_GPS = true;
            break;
          }
#endif
#if HOTT_SIM_VARIO_SENSOR
          if (addr == HOTT_TELEMETRY_VARIO_SENSOR_ID) {
            hott_send_msg((int8_t *)&hott_vario_msg, sizeof(struct HOTT_VARIO_MSG));
            HOTT_REQ_UPDATE_VARIO = true;
            break;
          }
#endif
          break; // HOTT_BINARY_MODE_REQUEST_ID:

        default:
          break;
      }
    } else {
      while (uart_char_available(&HOTT_PORT)) {
        uart_getch(&HOTT_PORT);
      }
      hott_serial_request_timer = 0;
    }
  }
}

static void hott_periodic_event(uint32_t tnow)
{
  static uint32_t hott_serial_timer;

  hott_check_serial_data(tnow);
  if (hott_msg_ptr == 0) { return; }
  if (hott_telemetry_is_sending) {
    if (tnow - hott_serial_timer < 3000) { //delay ca. 3,5 mS. 19200 baud = 520uS / int8_t + 3ms required delay
      return;
    }
  } else {
    tnow = get_sys_time_usec();
  }
  hott_send_telemetry_data();
  hott_serial_timer = tnow;
}

void hott_event(void)
{
  if (SysTimeTimer(hott_event_timer) > 1000) {
    SysTimeTimerStart(hott_event_timer);
    hott_periodic_event(hott_event_timer);
  }
}
