/*
 * Copyright (C) 2011-2014 Gautier Hattenberger <gautier.hattenberger@enac.fr>
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

/** \file subsystems/datalink/transport.h
 *  generic transport header
 */

#ifndef TRANSPORT_H
#define TRANSPORT_H

#include <inttypes.h>
#include "mcu_periph/link_device.h"
#include "std.h"

#ifndef TRANSPORT_PAYLOAD_LEN
#define TRANSPORT_PAYLOAD_LEN 256
#endif

/** Generic reception transport header
 */
struct transport_rx {
  uint8_t payload[TRANSPORT_PAYLOAD_LEN]; ///< payload buffer
  volatile uint8_t payload_len;           ///< payload buffer length
  volatile bool_t msg_received;           ///< message received flag
  uint8_t ovrn, error;                    ///< overrun and error flags
};

/** Data type
 */
enum TransportDataType {
  DL_TYPE_ARRAY_LENGTH,
  DL_TYPE_CHAR,
  DL_TYPE_UINT8,
  DL_TYPE_INT8,
  DL_TYPE_UINT16,
  DL_TYPE_INT16,
  DL_TYPE_UINT32,
  DL_TYPE_INT32,
  DL_TYPE_UINT64,
  DL_TYPE_INT64,
  DL_TYPE_FLOAT,
  DL_TYPE_DOUBLE,
  DL_TYPE_TIMESTAMP
};

/** Data format (scalar or array)
 */
enum TransportDataFormat {
  DL_FORMAT_SCALAR,
  DL_FORMAT_ARRAY
};

/** Function pointers definition
 *
 * they are used to cast the real functions with the correct type
 * to store in the transport structure
 */
typedef uint8_t (*size_of_t)(void *, uint8_t);
typedef int (*check_available_space_t)(void *, struct link_device *, uint8_t);
typedef void (*put_bytes_t)(void *, struct link_device *, enum TransportDataType, enum TransportDataFormat, uint8_t,
                            const void *);
typedef void (*put_named_byte_t)(void *, struct link_device *, enum TransportDataType, enum TransportDataFormat,
                                 uint8_t, const char *);
typedef void (*start_message_t)(void *, struct link_device *, uint8_t);
typedef void (*end_message_t)(void *, struct link_device *);
typedef void (*overrun_t)(void *, struct link_device *);
typedef void (*count_bytes_t)(void *, struct link_device *, uint8_t);

/** Generic transmission transport header
 */
struct transport_tx {
  size_of_t size_of;                              ///< get size of payload with transport header and trailer
  check_available_space_t check_available_space;  ///< check if transmit buffer is not full
  put_bytes_t put_bytes;                          ///< send bytes
  put_named_byte_t put_named_byte;                ///< send a single byte or its name
  start_message_t start_message;                  ///< transport header
  end_message_t end_message;                      ///< transport trailer
  overrun_t overrun;                              ///< overrun
  count_bytes_t count_bytes;                      ///< count bytes to send
  void *impl;                                     ///< pointer to parent implementation
};

#endif /* TRANSPORT_H */

