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

/** @file modules/telemetry/telemetry_intermcu_fbw.c
 *  @brief Telemetry through InterMCU
 */

#include "telemetry_intermcu.h"
#include "modules/intermcu/intermcu.h"
#include "pprzlink/pprz_transport.h"
#include "pprzlink/intermcu_msg.h"
#include "modules/datalink/telemetry.h"
#include "firmwares/rotorcraft/main_fbw.h"

#define MSG_SIZE 256

extern fbw_mode_enum fbw_mode;

/* Structure for handling telemetry over InterMCU */
struct telemetry_intermcu_t {
  struct link_device *dev;      ///< Device structure for communication
  struct pprz_transport trans;  ///< Transport without any extra encoding

  uint8_t rx_buffer[MSG_SIZE];  ///< Received bytes from datalink
  bool msg_received;            ///< Whenever a datalink message is received
};

/* Telemetry InterMCU throughput */
static struct telemetry_intermcu_t telemetry_intermcu;

/* Statically defined functions */
static void telemetry_intermcu_repack(struct transport_tx *trans, struct link_device *dev, uint8_t ac_id, uint8_t *msg, uint8_t size);

/* InterMCU initialization */
void telemetry_intermcu_init(void)
{
  // Initialize transport structure
  pprz_transport_init(&telemetry_intermcu.trans);

  // Set the link device
  telemetry_intermcu.dev = &(TELEMETRY_INTERMCU_DEV.device);
}

/* InterMCU periodic handling of telemetry */
void telemetry_intermcu_periodic(void)
{

}

/* InterMCU event handling of telemetry */
void telemetry_intermcu_event(void)
{
  pprz_check_and_parse(&(TELEMETRY_INTERMCU_DEV).device, &telemetry_intermcu.trans, telemetry_intermcu.rx_buffer, &telemetry_intermcu.msg_received);
  if(telemetry_intermcu.msg_received) {
    /* Switch on MSG ID */
    switch(pprzlink_get_msg_id(telemetry_intermcu.rx_buffer)) {
      case DL_EMERGENCY_CMD:
        if(DL_EMERGENCY_CMD_ac_id(telemetry_intermcu.rx_buffer) == AC_ID
            && DL_EMERGENCY_CMD_cmd(telemetry_intermcu.rx_buffer) == 0) {
          fbw_mode = FBW_MODE_FAILSAFE;
        }
        break;

      default:
        break;
    }

    /* Forward to AP */
    pprz_msg_send_IMCU_DATALINK(&(intermcu.transport.trans_tx), intermcu.device,
                            INTERMCU_FBW, telemetry_intermcu.trans.trans_rx.payload_len, telemetry_intermcu.rx_buffer);

    telemetry_intermcu.msg_received = false;
  }
}

void telemetry_intermcu_on_msg(uint8_t* msg, uint8_t size)
{
  telemetry_intermcu_repack(&(telemetry_intermcu.trans.trans_tx), telemetry_intermcu.dev, AC_ID, msg, size);
}

static void telemetry_intermcu_repack(struct transport_tx *trans, struct link_device *dev, uint8_t ac_id,uint8_t *msg, uint8_t size)
{
#if PPRZLINK_DEFAULT_VER == 2
  struct pprzlink_msg pmsg;
  pmsg.trans = trans;
  pmsg.dev = dev;
  pmsg.sender_id = ac_id;
  pmsg.receiver_id = 0;
  pmsg.component_id = 0;
  
  trans->count_bytes(&pmsg, size);
  trans->start_message(&pmsg, _FD, size);
  trans->put_bytes(&pmsg, _FD, DL_TYPE_UINT8, DL_FORMAT_ARRAY, (void *) msg, size);
  trans->end_message(&pmsg, _FD);
#else
  trans->count_bytes(trans->impl, dev, trans->size_of(trans->impl, size));
  trans->start_message(trans->impl, dev, 0, size);
  trans->put_bytes(trans->impl, dev, 0, DL_TYPE_UINT8, DL_FORMAT_ARRAY, (void *) msg, size);
  trans->end_message(trans->impl, dev, 0);
#endif
}
