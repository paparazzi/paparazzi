/*
 * Copyright (C) 2015 Freek van Tienen <freek.v.tienen@gmail.com>
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

/** @file modules/intermcu/intermcu_ap.c
 *  @brief Rotorcraft Inter-MCU on the autopilot
 */

#include "intermcu_ap.h"
#include "pprzlink/intermcu_msg.h"
#include "modules/radio_control/radio_control.h"
#include "mcu_periph/uart.h"

#include "modules/energy/electrical.h"
#include "autopilot.h"

#if COMMANDS_NB > 8
#error "INTERMCU UART CAN ONLY SEND 8 COMMANDS OR THE UART WILL BE OVERFILLED"
#endif


/* Main InterMCU defines */
struct intermcu_t intermcu = {
  .device = (&((INTERMCU_LINK).device)),
  .enabled = true,
  .msg_available = false,
};
uint8_t imcu_msg_buf[128] __attribute__((aligned));  ///< The InterMCU message buffer
static struct fbw_status_t fbw_status;
static inline void intermcu_parse_msg(void (*rc_frame_handler)(void));

#if IMCU_GPS
#include "std.h"
#include "modules/core/abi.h"
#include "modules/gps/gps.h"
static struct GpsState gps_imcu;
#endif


#if PERIODIC_TELEMETRY
#include "modules/datalink/telemetry.h"

/* Send FBW status */
static void send_status(struct transport_tx *trans, struct link_device *dev)
{
  pprz_msg_send_FBW_STATUS(trans, dev, AC_ID,
                           &fbw_status.rc_status, &fbw_status.frame_rate, &fbw_status.mode,
                           &fbw_status.electrical.vsupply, &fbw_status.electrical.current);
}
#endif

/* InterMCU initialization */
void intermcu_init(void)
{
  pprz_transport_init(&intermcu.transport);

#if IMCU_GPS
  gps_imcu.fix = GPS_FIX_NONE;
  gps_imcu.pdop = 0;
  gps_imcu.sacc = 0;
  gps_imcu.pacc = 0;
  gps_imcu.cacc = 0;
  gps_imcu.comp_id = GPS_IMCU_ID;
#endif

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

#if IMCU_GPS
  RunOnceEvery(PERIODIC_FREQUENCY, gps_periodic_check(&gps_imcu));
#endif
}

/* Enable or disable the communication of the InterMCU */
void intermcu_set_enabled(bool value)
{
  intermcu.enabled = value;
}

/* Send the actuators to the FBW */
void intermcu_set_actuators(pprz_t *command_values, uint8_t ap_mode __attribute__((unused)))
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
                              INTERMCU_AP, &intermcu.cmd_status, COMMANDS_NB, command_values); //TODO: Append more status
  intermcu.cmd_status = 0;
}

/* Send the spektrum Bind message */
void intermcu_send_spektrum_bind(void)
{
  if (intermcu.enabled) {
    pprz_msg_send_IMCU_SPEKTRUM_SOFT_BIND(&(intermcu.transport.trans_tx), intermcu.device, INTERMCU_AP);
  }
}

/* Parse incomming InterMCU messages */
#pragma GCC diagnostic ignored "-Wcast-align"
static inline void intermcu_parse_msg(void (*rc_frame_handler)(void))
{
  /* Parse the Inter MCU message */
  uint8_t msg_id = pprzlink_get_msg_id(imcu_msg_buf);
#if PPRZLINK_DEFAULT_VER == 2
  // Check that the message is really a intermcu message
  if (pprzlink_get_msg_class_id(imcu_msg_buf) == DL_intermcu_CLASS_ID) {
#endif
    switch (msg_id) {
      case DL_IMCU_RADIO_COMMANDS: {
        uint8_t i;
        uint8_t size = DL_IMCU_RADIO_COMMANDS_values_length(imcu_msg_buf);
        intermcu.status = DL_IMCU_RADIO_COMMANDS_status(imcu_msg_buf);
        for (i = 0; i < size; i++) {
          radio_control.values[i] = DL_IMCU_RADIO_COMMANDS_values(imcu_msg_buf)[i];
        }

        radio_control.frame_cpt++;
        radio_control.time_since_last_frame = 0;
        radio_control.status = RC_OK;
        rc_frame_handler();
        break;
      }

      case DL_IMCU_FBW_STATUS: {
        fbw_status.rc_status = DL_IMCU_FBW_STATUS_rc_status(imcu_msg_buf);
        fbw_status.frame_rate = DL_IMCU_FBW_STATUS_frame_rate(imcu_msg_buf);
        fbw_status.mode = DL_IMCU_FBW_STATUS_mode(imcu_msg_buf);
        fbw_status.electrical.vsupply = DL_IMCU_FBW_STATUS_vsupply(imcu_msg_buf);
        fbw_status.electrical.current = DL_IMCU_FBW_STATUS_current(imcu_msg_buf);
        break;
      }

  #if TELEMETRY_INTERMCU
      case DL_IMCU_DATALINK: {
        uint8_t size = DL_IMCU_DATALINK_msg_length(imcu_msg_buf);
        uint8_t *msg = DL_IMCU_DATALINK_msg(imcu_msg_buf);
        telemetry_intermcu_on_msg(msg, size);
        break;
      }
  #endif

  #if IMCU_GPS
      case DL_IMCU_REMOTE_GPS: {
        uint32_t now_ts = get_sys_time_usec();
        gps_imcu.ecef_pos.x = DL_IMCU_REMOTE_GPS_ecef_x(imcu_msg_buf);
        gps_imcu.ecef_pos.y = DL_IMCU_REMOTE_GPS_ecef_y(imcu_msg_buf);
        gps_imcu.ecef_pos.z = DL_IMCU_REMOTE_GPS_ecef_z(imcu_msg_buf);
        SetBit(gps_imcu.valid_fields, GPS_VALID_POS_ECEF_BIT);

        gps_imcu.lla_pos.alt = DL_IMCU_REMOTE_GPS_alt(imcu_msg_buf);
        gps_imcu.hmsl = DL_IMCU_REMOTE_GPS_hmsl(imcu_msg_buf);
        SetBit(gps_imcu.valid_fields, GPS_VALID_HMSL_BIT);

        gps_imcu.ecef_vel.x = DL_IMCU_REMOTE_GPS_ecef_xd(imcu_msg_buf);
        gps_imcu.ecef_vel.y = DL_IMCU_REMOTE_GPS_ecef_yd(imcu_msg_buf);
        gps_imcu.ecef_vel.z = DL_IMCU_REMOTE_GPS_ecef_zd(imcu_msg_buf);
        SetBit(gps_imcu.valid_fields, GPS_VALID_VEL_ECEF_BIT);

        gps_imcu.course = DL_IMCU_REMOTE_GPS_course(imcu_msg_buf);
        gps_imcu.gspeed = DL_IMCU_REMOTE_GPS_gspeed(imcu_msg_buf);
        SetBit(gps_imcu.valid_fields, GPS_VALID_COURSE_BIT);

        gps_imcu.pacc = DL_IMCU_REMOTE_GPS_pacc(imcu_msg_buf);
        gps_imcu.sacc = DL_IMCU_REMOTE_GPS_sacc(imcu_msg_buf);
        gps_imcu.num_sv = DL_IMCU_REMOTE_GPS_numsv(imcu_msg_buf);
        gps_imcu.fix = DL_IMCU_REMOTE_GPS_fix(imcu_msg_buf);

        // set gps msg time
        gps_imcu.last_msg_ticks = sys_time.nb_sec_rem;
        gps_imcu.last_msg_time = sys_time.nb_sec;

        if (gps_imcu.fix >= GPS_FIX_3D) {
          gps_imcu.last_3dfix_ticks = sys_time.nb_sec_rem;
          gps_imcu.last_3dfix_time = sys_time.nb_sec;
        }

        AbiSendMsgGPS(GPS_IMCU_ID, now_ts, &gps_imcu);
        break;
      }

  #endif

      default:
        break;
    }
#if PPRZLINK_DEFAULT_VER == 2
  }
#endif
}
#pragma GCC diagnostic pop

/* Radio control event misused as InterMCU event for frame_handler */
void RadioControlEvent(void (*frame_handler)(void))
{
  /* Parse incoming bytes */
  if (intermcu.enabled) {
    pprz_check_and_parse(intermcu.device, &intermcu.transport, imcu_msg_buf, &intermcu.msg_available);

    if (intermcu.msg_available) {
      intermcu_parse_msg(frame_handler);
      intermcu.msg_available = false;
    }
  }
}
