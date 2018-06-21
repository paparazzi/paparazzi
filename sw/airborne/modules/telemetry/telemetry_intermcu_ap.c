/*
 * Copyright (C) 2016 Freek van Tienen <freek.v.tienen@gmail.com>
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

/** @file modules/telemetry/telemetry_intermcu_ap.c
 *  @brief Telemetry through InterMCU
 */

#define PERIODIC_C_INTERMCU
#include "telemetry_intermcu.h"
#include "telemetry_intermcu_ap.h"
#include "subsystems/intermcu.h"
#include "pprzlink/intermcu_msg.h"
#include "pprzlink/short_transport.h"
#include "generated/periodic_telemetry.h"
#include "subsystems/datalink/telemetry.h"
#include "subsystems/datalink/datalink.h"

/* Telemetry InterMCU throughput */
struct telemetry_intermcu_t telemetry_intermcu;

/* Static functions */
static bool telemetry_intermcu_check_free_space(struct telemetry_intermcu_t *p, long *fd __attribute__((unused)), uint16_t len);
static void telemetry_intermcu_put_byte(struct telemetry_intermcu_t *p, long fd __attribute__((unused)), uint8_t data);
static void telemetry_intermcu_put_buffer(struct telemetry_intermcu_t *p, long fd, uint8_t *data, uint16_t len);
static void telemetry_intermcu_send_message(struct telemetry_intermcu_t *p, long fd __attribute__((unused)));

/* InterMCU initialization */
void telemetry_intermcu_init(void)
{
  // Initialize transport structure
  short_transport_init(&telemetry_intermcu.trans);

  // Configure the device
  telemetry_intermcu.dev.check_free_space = (check_free_space_t)telemetry_intermcu_check_free_space;
  telemetry_intermcu.dev.put_byte = (put_byte_t)telemetry_intermcu_put_byte;
  telemetry_intermcu.dev.put_buffer = (put_buffer_t)telemetry_intermcu_put_buffer;
  telemetry_intermcu.dev.send_message = (send_message_t)telemetry_intermcu_send_message;
  telemetry_intermcu.dev.periph = (void *)&telemetry_intermcu;
}

/* InterMCU periodic handling of telemetry */
void telemetry_intermcu_periodic(void)
{
  periodic_telemetry_send_InterMCU(DefaultPeriodic, &telemetry_intermcu.trans.trans_tx, &telemetry_intermcu.dev);
}

/* InterMCU event handling of telemetry */
void telemetry_intermcu_event(void)
{

}

void telemetry_intermcu_on_msg(uint8_t* msg, uint8_t size __attribute__((unused)))
{
  datalink_time = 0;
  datalink_nb_msgs++;
  dl_parse_msg(&telemetry_intermcu.dev, &telemetry_intermcu.trans.trans_tx, msg);
}

static bool telemetry_intermcu_check_free_space(struct telemetry_intermcu_t *p, long *fd __attribute__((unused)), uint16_t len)
{
  return ((p->buf_idx + len) < (TELEMERTY_INTERMCU_MSG_SIZE - 1));
}

static void telemetry_intermcu_put_byte(struct telemetry_intermcu_t *p, long fd __attribute__((unused)), uint8_t data)
{
  if(p->buf_idx >= (TELEMERTY_INTERMCU_MSG_SIZE - 1))
    return;

  p->buf[p->buf_idx++] = data;
}

static void telemetry_intermcu_put_buffer(struct telemetry_intermcu_t *p, long fd __attribute__((unused)), uint8_t *data, uint16_t len)
{
  int i;
  for (i = 0; i < len; i++) {
    p->buf[p->buf_idx++] = data[i];
  }
}

static void telemetry_intermcu_send_message(struct telemetry_intermcu_t *p, long fd __attribute__((unused)))
{
  pprz_msg_send_IMCU_TELEMETRY(&(intermcu.transport.trans_tx), intermcu.device,
                            INTERMCU_AP, p->buf_idx, p->buf);
  p->buf_idx = 0;
}
