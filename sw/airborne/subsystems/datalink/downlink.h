/*
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

#ifndef PPRZ_DATALINK_EXPORT

#include "generated/modules.h"
#include "messages.h"
#include "generated/airframe.h" // AC_ID is required

#if defined SITL

#ifdef SIM_UART
#include "sim_uart.h"
#include "subsystems/datalink/pprz_transport.h"
#include "subsystems/datalink/xbee.h"
#else /* SIM_UART */
/** Software In The Loop simulation uses IVY bus directly as the transport layer */
#include "ivy_transport.h"
#endif

#else /** SITL */

#include "subsystems/datalink/udp.h"
#include "subsystems/datalink/pprz_transport.h"
#include "subsystems/datalink/xbee.h"
#include "subsystems/datalink/w5100.h"
#if USE_SUPERBITRF
#include "subsystems/datalink/superbitrf.h"
#endif
#if USE_AUDIO_TELEMETRY
#include "subsystems/datalink/audio_telemetry.h"
#endif
#ifdef USE_USB_SERIAL
#include "mcu_periph/usb_serial.h"
#endif
#include "mcu_periph/uart.h"

#endif /** !SITL */

#else /* PPRZ_DATALINK_EXPORT defined */

#include "messages.h"
#include "pprz_transport.h"
#ifndef AC_ID
#define AC_ID 0
#endif

#endif

#ifndef DefaultChannel
#define DefaultChannel DOWNLINK_TRANSPORT
#endif

// FIXME are DOWNLINK_AP|FBW_DEVICE distinction really necessary ?
// by default use AP_DEVICE if nothing is set ?
#ifndef DOWNLINK_DEVICE
#define DOWNLINK_DEVICE DOWNLINK_AP_DEVICE
#endif

#ifndef DefaultDevice
#define DefaultDevice DOWNLINK_DEVICE
#endif

/** Counter of messages not sent because of unavailibity of the output buffer*/
extern uint8_t downlink_nb_ovrn;
extern uint16_t downlink_nb_bytes;
extern uint16_t downlink_nb_msgs;

/* Transport macros
 *
 * call transport functions from channel
 */
#define __Transport(dev, _x) dev##_x
#define _Transport(dev, _x) __Transport(dev, _x)
#define Transport(_chan, _fun) _Transport(_chan, _fun)


/** Set of macros for generated code (messages.h) from messages.xml */
/** 2 = ac_id + msg_id */
#define DownlinkIDsSize(_trans, _dev, _x) (_x+2)
#define DownlinkSizeOf(_trans, _dev, _x) Transport(_trans, SizeOf(_dev, DownlinkIDsSize(_trans, _dev, _x)))

#define DownlinkCheckFreeSpace(_trans, _dev, _x) Transport(_trans, CheckFreeSpace(_dev, (uint8_t)(_x)))

#define DownlinkPutUint8(_trans, _dev, _x) Transport(_trans, PutUint8(_dev, _x))

#define DownlinkPutInt8ByAddr(_trans, _dev, _x) Transport(_trans, PutInt8ByAddr(_dev, _x))
#define DownlinkPutUint8ByAddr(_trans, _dev, _x) Transport(_trans, PutUint8ByAddr(_dev, _x))
#define DownlinkPutInt16ByAddr(_trans, _dev, _x) Transport(_trans, PutInt16ByAddr(_dev, _x))
#define DownlinkPutUint16ByAddr(_trans, _dev, _x) Transport(_trans, PutUint16ByAddr(_dev, _x))
#define DownlinkPutInt32ByAddr(_trans, _dev, _x) Transport(_trans, PutInt32ByAddr(_dev, _x))
#define DownlinkPutUint32ByAddr(_trans, _dev, _x) Transport(_trans, PutUint32ByAddr(_dev, _x))
#define DownlinkPutFloatByAddr(_trans, _dev, _x) Transport(_trans, PutFloatByAddr(_dev, _x))

#define DownlinkPutDoubleByAddr(_trans, _dev, _x) Transport(_trans, PutDoubleByAddr(_dev, _x))
#define DownlinkPutUint64ByAddr(_trans, _dev, _x) Transport(_trans, PutUint64ByAddr(_dev, _x))
#define DownlinkPutInt64ByAddr(_trans, _dev, _x) Transport(_trans, PutInt64ByAddr(_dev, _x))
#define DownlinkPutCharByAddr(_trans, _dev, _x) Transport(_trans, PutCharByAddr(_dev, _x))

#define DownlinkPutFloatArray(_trans, _dev, _n, _x) Transport(_trans, PutFloatArray(_dev, _n, _x))
#define DownlinkPutDoubleArray(_trans, _dev, _n, _x) Transport(_trans, PutDoubleArray(_dev, _n, _x))
#define DownlinkPutInt16Array(_trans, _dev, _n, _x) Transport(_trans, PutInt16Array(_dev, _n, _x))
#define DownlinkPutUint16Array(_trans, _dev, _n, _x) Transport(_trans, PutUint16Array(_dev, _n, _x))
#define DownlinkPutInt32Array(_trans, _dev, _n, _x) Transport(_trans, PutInt32Array(_dev, _n, _x))
#define DownlinkPutUint32Array(_trans, _dev, _n, _x) Transport(_trans, PutUint32Array(_dev, _n, _x))
#define DownlinkPutInt64Array(_trans, _dev, _n, _x) Transport(_trans, PutInt64Array(_dev, _n, _x))
#define DownlinkPutUint64Array(_trans, _dev, _n, _x) Transport(_trans, PutUint64Array(_dev, _n, _x))
#define DownlinkPutInt8Array(_trans, _dev, _n, _x) Transport(_trans, PutInt8Array(_dev, _n, _x))
#define DownlinkPutUint8Array(_trans, _dev, _n, _x) Transport(_trans, PutUint8Array(_dev, _n, _x))
#define DownlinkPutCharArray(_trans, _dev, _n, _x) Transport(_trans, PutCharArray(_dev, _n, _x))

#define DownlinkPutFloatFixedArray(_trans, _dev, _n, _x) Transport(_trans, PutFloatFixedArray(_dev, _n, _x))
#define DownlinkPutDoubleFixedArray(_trans, _dev, _n, _x) Transport(_trans, PutDoubleFixedArray(_dev, _n, _x))
#define DownlinkPutInt16FixedArray(_trans, _dev, _n, _x) Transport(_trans, PutInt16FixedArray(_dev, _n, _x))
#define DownlinkPutUint16FixedArray(_trans, _dev, _n, _x) Transport(_trans, PutUint16FixedArray(_dev, _n, _x))
#define DownlinkPutInt32FixedArray(_trans, _dev, _n, _x) Transport(_trans, PutInt32FixedArray(_dev, _n, _x))
#define DownlinkPutUint32FixedArray(_trans, _dev, _n, _x) Transport(_trans, PutUint32FixedArray(_dev, _n, _x))
#define DownlinkPutInt64FixedArray(_trans, _dev, _n, _x) Transport(_trans, PutInt64FixedArray(_dev, _n, _x))
#define DownlinkPutUint64FixedArray(_trans, _dev, _n, _x) Transport(_trans, PutUint64FixedArray(_dev, _n, _x))
#define DownlinkPutInt8FixedArray(_trans, _dev, _n, _x) Transport(_trans, PutInt8FixedArray(_dev, _n, _x))
#define DownlinkPutUint8FixedArray(_trans, _dev, _n, _x) Transport(_trans, PutUint8FixedArray(_dev, _n, _x))
#define DownlinkPutCharFixedArray(_trans, _dev, _n, _x) Transport(_trans, PutCharFixedArray(_dev, _n, _x))

#define DownlinkOverrun(_trans, _dev) downlink_nb_ovrn++;
#define DownlinkCountBytes(_trans, _dev, _n) downlink_nb_bytes += _n;

#define DownlinkStartMessage(_trans, _dev, _name, msg_id, payload_len) { \
  downlink_nb_msgs++; \
  Transport(_trans, Header(_dev, DownlinkIDsSize(_trans, _dev, payload_len))); \
  Transport(_trans, PutUint8(_dev, AC_ID)); \
  Transport(_trans, PutNamedUint8(_dev, _name, msg_id)); \
}

#define DownlinkEndMessage(_trans, _dev) Transport(_trans, Trailer(_dev))

#endif /* DOWNLINK_H */
