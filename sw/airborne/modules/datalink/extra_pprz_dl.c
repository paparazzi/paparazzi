/*
 * Copyright (C) 2006  Pascal Brisset, Antoine Drouin
 * Copyright (C) 2010  ENAC
 * Copyright (C) 2016  2016 Michal Podhradsky <http://github.com/podhrmic>
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
 * @file "modules/datalink/extra_pprz_dl.c"
 * Extra datalink and telemetry using PPRZ protocol
 *
 * NOTES (for future reference):
 * This note is not needed unless we want to define our own messages - in such case you would define the two
 * structs below and then define individual messages and registered them in the same way as
 * periodic telemetry.
 * struct telemetry_cb_slots telemetry_cbs_logger[TELEMETRY_PPRZ_NB_MSG] = TELEMETRY_PPRZ_CBS;
 * struct periodic_telemetry logger_telemetry = { TELEMETRY_PPRZ_NB_MSG, telemetry_cbs_logger };
 *
 * The registration should be done in the init function:
 * register_periodic_telemetry(&extra_telemetry, PPRZ_MSG_ID_xxx, send_xxx_message);
 *
 * Two extra notes for the periodic function:
 * 1) this sends registered messages from ExtraTelemetry process (as mentioned above) over dedicated port do:
 *    periodic_telemetry_send_ExtraTelemetry(&extra_telemetry, &pprz_tp_extra.trans_tx, &(EXTRA_TELEMETRY_PORT).device);
 * 2) to send ExtraTelemetry messages over default channel just change to:
 *    periodic_telemetry_send_ExtraTelemetry(DefaultPeriodic, &(DefaultChannel).trans_tx, &(DefaultDevice).device);
 */
#define PERIODIC_C_EXTRA

#include "modules/datalink/extra_pprz_dl.h"
#include "subsystems/datalink/telemetry.h"
#include "subsystems/datalink/datalink.h"
#include "pprzlink/pprz_transport.h"

#if USE_UDP
#include "mcu_periph/udp.h"
#endif

#if USE_USB_SERIAL
#include "mcu_periph/usb_serial.h"
#endif


bool extra_dl_msg_available;
uint8_t extra_dl_buffer[MSG_SIZE]  __attribute__((aligned));

struct pprz_transport extra_pprz_tp;

void extra_pprz_dl_init(void)
{
  pprz_transport_init(&extra_pprz_tp);
}

void extra_pprz_dl_event(void)
{
  pprz_check_and_parse(&EXTRA_DOWNLINK_DEVICE.device, &extra_pprz_tp, extra_dl_buffer, &extra_dl_msg_available);
  DlCheckAndParse(&EXTRA_DOWNLINK_DEVICE.device, &extra_pprz_tp.trans_tx, extra_dl_buffer, &extra_dl_msg_available);
}

void extra_pprz_dl_periodic(void)
{
#if PERIODIC_TELEMETRY && defined(TELEMETRY_PROCESS_Extra)
  // send periodic messages as defined in the Extra process, we are using DefaultPeriodic so we can send standard messages
  periodic_telemetry_send_Extra(DefaultPeriodic, &extra_pprz_tp.trans_tx, &(EXTRA_DOWNLINK_DEVICE).device);
#endif
}


void extra_pprz_dl_parse_payload_cmd(void)
{
  // check if the command it meant for me
  if (AC_ID != DL_PAYLOAD_COMMAND_ac_id(extra_dl_buffer)){
    return;
  }

  // check if the payload length it correct
  if (EXPECTED_PAYLOAD_LENGTH != DL_PAYLOAD_COMMAND_command_length(extra_dl_buffer)){
    return;
  }

  // optionally we can check for PAYLOAD_COMMAND version etc, depending on what we define in the packet
  if (DL_PAYLOAD_COMMAND_command(extra_dl_buffer)[PAYLOAD_CMD_IDX] == PAYLOAD_CMD_INFO){
    // TODO: do something
  }
}
