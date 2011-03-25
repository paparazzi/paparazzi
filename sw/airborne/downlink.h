/*
 * Paparazzi $Id$
 *
 * Copyright (C) 2003-2006  Pascal Brisset, Antoine Drouin
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

/** \file downlink.h
 *  \brief Common code for AP and FBW telemetry
 *
 */

#ifndef DOWNLINK_H
#define DOWNLINK_H

#include <inttypes.h>

#include "generated/modules.h"
#include "messages.h"
#include "generated/airframe.h" // AC_ID is required

#if defined SITL

#ifdef SIM_UART
#include "sim_uart.h"
#include "pprz_transport.h"
#include "xbee.h"
#else /* SIM_UART */
/** Software In The Loop simulation uses IVY bus directly as the transport layer */
#include "ivy_transport.h"
#endif

#else /** SITL */
#include "pprz_transport.h"
#include "modem.h"
#include "xbee.h"
#ifdef USE_USB_SERIAL
#include "mcu_periph/usb_serial.h"
#endif
#endif /** !SITL */

#ifndef DefaultChannel
#define DefaultChannel DOWNLINK_TRANSPORT
#endif

#ifdef AP
/** Telemetry mode for AP process: index in the telemetry.xml file */
extern uint8_t telemetry_mode_Ap_DefaultChannel;
#endif

#ifdef FBW
/** Telemetry mode for FBW process: index in the telemetry.xml file */
extern uint8_t telemetry_mode_Fbw_DefaultChannel;
#endif

/** Counter of messages not sent because of unavailibity of the output buffer*/
extern uint8_t downlink_nb_ovrn;
extern uint16_t downlink_nb_bytes;
extern uint16_t downlink_nb_msgs;


#define __Transport(dev, _x) dev##_x
#define _Transport(dev, _x) __Transport(dev, _x)
#define Transport(_chan, _fun) _Transport(_chan, _fun)


/** Set of macros for generated code (messages.h) from messages.xml */
/** 2 = ac_id + msg_id */
#define DownlinkIDsSize(_chan, _x) (_x+2)
#define DownlinkSizeOf(_chan, _x) Transport(_chan, SizeOf(DownlinkIDsSize(_chan, _x)))

#define DownlinkCheckFreeSpace(_chan, _x) Transport(_chan, CheckFreeSpace((uint8_t)(_x)))

#define DownlinkPutUint8(_chan, _x) Transport(_chan, PutUint8(_x))

#define DownlinkPutInt8ByAddr(_chan, _x) Transport(_chan, PutInt8ByAddr(_x))
#define DownlinkPutUint8ByAddr(_chan, _x) Transport(_chan, PutUint8ByAddr(_x))
#define DownlinkPutInt16ByAddr(_chan, _x) Transport(_chan, PutInt16ByAddr(_x))
#define DownlinkPutUint16ByAddr(_chan, _x) Transport(_chan, PutUint16ByAddr(_x))
#define DownlinkPutInt32ByAddr(_chan, _x) Transport(_chan, PutInt32ByAddr(_x))
#define DownlinkPutUint32ByAddr(_chan, _x) Transport(_chan, PutUint32ByAddr(_x))
#define DownlinkPutFloatByAddr(_chan, _x) Transport(_chan, PutFloatByAddr(_x))

#define DownlinkPutDoubleByAddr(_chan, _x) Transport(_chan, PutDoubleByAddr(_x))

#define DownlinkPutFloatArray(_chan, _n, _x) Transport(_chan, PutFloatArray(_n, _x))
#define DownlinkPutDoubleArray(_chan, _n, _x) Transport(_chan, PutDoubleArray(_n, _x))
#define DownlinkPutInt16Array(_chan, _n, _x) Transport(_chan, PutInt16Array(_n, _x))
#define DownlinkPutUint16Array(_chan, _n, _x) Transport(_chan, PutUint16Array(_n, _x))
#define DownlinkPutInt32Array(_chan, _n, _x) Transport(_chan, PutInt32Array(_n, _x))
#define DownlinkPutUint32Array(_chan, _n, _x) Transport(_chan, PutUint32Array(_n, _x))
#define DownlinkPutUint8Array(_chan, _n, _x) Transport(_chan, PutUint8Array(_n, _x))

#define DownlinkOverrun(_chan) downlink_nb_ovrn++;
#define DownlinkCountBytes(_chan, _n) downlink_nb_bytes += _n;

#define DownlinkStartMessage(_chan, _name, msg_id, payload_len) { \
  downlink_nb_msgs++; \
  Transport(_chan, Header(DownlinkIDsSize(_chan, payload_len))); \
  Transport(_chan, PutUint8(AC_ID)); \
  Transport(_chan, PutNamedUint8(_name, msg_id)); \
}

#define DownlinkEndMessage(_chan) Transport(_chan, Trailer())

#endif /* DOWNLINK_H */
