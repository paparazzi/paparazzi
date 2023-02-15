/*
 * Copyright (C) 2014 Freek van Tienen <freek.v.tienen@gmail.com>
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

/** @file modules/datalink/mavlink.h
 *  @brief Basic MAVLink datalink implementation
 */

#ifndef DATALINK_MAVLINK_H
#define DATALINK_MAVLINK_H

#define MAVLINK_USE_CONVENIENCE_FUNCTIONS
#define MAVLINK_ALIGNED_FIELDS 0

#include <mavlink/mavlink_types.h>

#if USE_UDP
#include "mcu_periph/udp.h"
#endif
#if USE_USB_SERIAL || USE_USB_SERIAL_DEBUG
#include "mcu_periph/usb_serial.h"
#endif
#include "mcu_periph/uart.h"

#ifndef MAVLINK_DEBUG
#define MAVLINK_DEBUG(...) {}
#endif

#if MAVLINK_DEBUG == printf
#include <stdio.h>
#endif

/*
 * MAVLink description before main MAVLink include
 */
extern mavlink_system_t mavlink_system;

#ifndef MAVLINK_DEV
#define MAVLINK_DEV uart1
#endif

/*
 * The MAVLink link description
 */
#define MAVLinkDev (&(MAVLINK_DEV).device)
#define MAVLinkTransmit(c) MAVLinkDev->put_byte(MAVLinkDev->periph, 0, c)
#define MAVLinkChAvailable() MAVLinkDev->char_available(MAVLinkDev->periph)
#define MAVLinkGetch() MAVLinkDev->get_byte(MAVLinkDev->periph)
#define MAVLinkSendMessage() MAVLinkDev->send_message(MAVLinkDev->periph, 0)

/**
 * Module functions
 */
void mavlink_init(void);
void mavlink_periodic(void);
void mavlink_periodic_telemetry(void);
void mavlink_event(void);

/**
 * @brief Send one char (uint8_t) over a comm channel
 *
 * @param chan MAVLink channel to use, usually MAVLINK_COMM_0 = UART0
 * @param ch Character to send
 */
static inline void comm_send_ch(mavlink_channel_t chan __attribute__((unused)), uint8_t ch)
{
  // Send bytes
  MAVLinkTransmit(ch);
}

#endif // DATALINK_MAVLINK_H
