/*
 * Copyright (C) 2015 Freek van Tienen <freek.v.tienen@gmail.com>
 * Copyright (C) 2022 Gautier Hattenberger <gautier.hattenberger@enac.fr>
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
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 *
 */

/** @file modules/intermcu/intermcu_ap.c
 *  @brief Inter-MCU on the AP side
 */

#define PERIODIC_C_INTERMCU

#include "modules/intermcu/intermcu_ap.h"
#include "pprzlink/intermcu_msg.h"
#include "modules/energy/electrical.h"
#include "generated/modules.h"
#include "autopilot.h"
#if TELEMETRY_INTERMCU
#include "modules/datalink/intermcu_dl.h"
#endif
#include "generated/periodic_telemetry.h"

#if COMMANDS_NB > 8
#warning "INTERMCU UART CAN ONLY SEND 8 COMMANDS OR THE UART MIGHT BE OVERFILLED"
#endif


/* Main InterMCU defines */
struct intermcu_t intermcu = {
  .device = (&((INTERMCU_LINK).device)),
  .enabled = true,
  .msg_available = false,
};

uint8_t imcu_msg_buf[128] __attribute__((aligned));  ///< The InterMCU message buffer

static struct fbw_status_t fbw_status;


#if PERIODIC_TELEMETRY
#include "modules/datalink/telemetry.h"

/* Send FBW status */
static void send_status(struct transport_tx *trans, struct link_device *dev)
{
  pprz_msg_send_FBW_STATUS(trans, dev, AC_ID,
                           &fbw_status.rc_status, &fbw_status.frame_rate, &fbw_status.mode,
                           &electrical.vsupply, &electrical.current);
}
#endif

/* InterMCU initialization */
void intermcu_init(void)
{
  pprz_transport_init(&intermcu.transport);

#if PERIODIC_TELEMETRY
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_FBW_STATUS, send_status);
#endif
}

/* Check for InterMCU loss */
void intermcu_periodic(void)
{
  /* Check for interMCU loss */
  if (intermcu.time_since_last_frame >= INTERMCU_LOST_CNT) {
    intermcu.status = INTERMCU_LOST;
  } else {
    intermcu.time_since_last_frame++;
  }

#ifdef TELEMETRY_PROCESS_InterMCU
  // send periodic InterMCU if defined in telemetry file
  periodic_telemetry_send_InterMCU(DefaultPeriodic, &intermcu.transport.trans_tx, intermcu.device);
#endif
}

/* Check new characters */
void intermcu_event(void)
{
  /* Parse incoming bytes */
  if (intermcu.enabled) {
    pprz_check_and_parse(intermcu.device, &intermcu.transport, imcu_msg_buf, &intermcu.msg_available);
    if (intermcu.msg_available) {
      uint8_t class_id = pprzlink_get_msg_class_id(imcu_msg_buf);
      // reset intermcu timer, don't touch msg_available flag
      intermcu.time_since_last_frame = 0;

      if (class_id == DL_intermcu_CLASS_ID) {
        // parse intermcu messages and call callbacks
        dl_parse_msg(intermcu.device, &intermcu.transport.trans_tx, imcu_msg_buf);
      } else {
        // reset datalink_time if message is not intermcu class
        datalink_time = 0;
        datalink_nb_msgs++;
#if TELEMETRY_INTERMCU
        // forward all other messages if needed
        intermcu_dl_on_msg(imcu_msg_buf, intermcu.transport.trans_rx.payload_len);
#endif
      }
      intermcu.msg_available = false;
    }
  }
}

/* Enable or disable the communication of the InterMCU */
void intermcu_set_enabled(bool value)
{
  intermcu.enabled = value;
}

/* Send the commands to the FBW */
void intermcu_send_commands(pprz_t *command_values, uint8_t ap_mode __attribute__((unused)))
{
  if (!intermcu.enabled) {
    return;
  }

  // Set the autopilot motors on status
  if (autopilot_get_motors_on()) {
    INTERMCU_SET_CMD_STATUS(INTERMCU_CMD_MOTORS_ON);
  }

  // Send the message and reset cmd_status
  pprz_msg_send_IMCU_COMMANDS(&(intermcu.transport.trans_tx), intermcu.device,
      AC_ID, &intermcu.cmd_status, COMMANDS_NB, command_values); //TODO: Append more status
  intermcu.cmd_status = 0;
}

/* Send the spektrum Bind message */
void intermcu_send_spektrum_bind(void)
{
  if (intermcu.enabled) {
    pprz_msg_send_IMCU_SPEKTRUM_SOFT_BIND(&(intermcu.transport.trans_tx), intermcu.device, AC_ID);
  }
}

void intermcu_parse_IMCU_FBW_STATUS(uint8_t *buf)
{
  fbw_status.rc_status = DL_IMCU_FBW_STATUS_rc_status(buf);
  fbw_status.frame_rate = DL_IMCU_FBW_STATUS_frame_rate(buf);
  fbw_status.mode = DL_IMCU_FBW_STATUS_mode(buf);
  electrical.vsupply = DL_IMCU_FBW_STATUS_vsupply(buf);
  electrical.current = DL_IMCU_FBW_STATUS_current(buf);
}

