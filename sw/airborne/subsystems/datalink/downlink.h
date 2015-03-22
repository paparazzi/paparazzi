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
/** Software In The Loop simulation uses IVY bus directly as the transport layer */
#include "ivy_transport.h"

#else /** SITL */

#include "subsystems/datalink/pprz_transport.h"
#include "subsystems/datalink/pprzlog_transport.h"
#include "subsystems/datalink/xbee.h"
#include "subsystems/datalink/w5100.h"
#if USE_SUPERBITRF
#include "subsystems/datalink/superbitrf.h"
#endif
#if USE_AUDIO_TELEMETRY
#include "subsystems/datalink/audio_telemetry.h"
#endif
#if USE_USB_SERIAL
#include "mcu_periph/usb_serial.h"
#endif
#ifdef USE_UDP
#include "mcu_periph/udp.h"
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

/** Downlink structure */
struct downlink {
  uint8_t nb_ovrn;    ///< Counter of messages not sent because of unavailibity of the output buffer
  uint16_t nb_bytes;  ///< Number of bytes send over telemetry
  uint16_t nb_msgs;   ///< Number of messages send over telemetry
};

extern struct downlink downlink;

// Init function
extern void downlink_init(void);

#endif /* DOWNLINK_H */
