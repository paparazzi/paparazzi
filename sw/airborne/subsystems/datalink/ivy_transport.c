/*
 * Copyright (C) 2003  Pascal Brisset, Antoine Drouin
 * Copyright (C) 2014  Gautier Hattenberger <gautier.hattenberger@enac.fr>
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

/**
 * @file subsystems/datalink/ivy_transport.c
 *
 * Building Paparazzi frames over IVY.
 *
 */

#include "std.h"
#include "subsystems/datalink/ivy_transport.h"
#include "subsystems/datalink/downlink.h"
#include "subsystems/datalink/transport.h"

#include <stdio.h>
#include <Ivy/ivy.h>

struct ivy_transport ivy_tp;

static void put_bytes(struct ivy_transport *trans, struct link_device *dev __attribute__((unused)),
                      enum TransportDataType type __attribute__((unused)), enum TransportDataFormat format __attribute__((unused)),
                      uint8_t len, const void *bytes)
{
  const uint8_t *b = (const uint8_t *) bytes;

  // Start delimiter "quote" for char arrays (strings)
  if (format == DL_FORMAT_ARRAY && type == DL_TYPE_CHAR) {
    trans->ivy_p += sprintf(trans->ivy_p, "\"");
  }

  int i = 0;
  while (i < len) {
    // print data with correct type
    switch (type) {
      case DL_TYPE_CHAR:
        trans->ivy_p += sprintf(trans->ivy_p, "%c", (char)(*((char *)(b + i))));
        i++;
        break;
      case DL_TYPE_UINT8:
        trans->ivy_p += sprintf(trans->ivy_p, "%u", b[i]);
        i++;
        break;
      case DL_TYPE_UINT16:
        trans->ivy_p += sprintf(trans->ivy_p, "%u", (uint16_t)(*((uint16_t *)(b + i))));
        i += 2;
        break;
      case DL_TYPE_UINT32:
      case DL_TYPE_TIMESTAMP:
        trans->ivy_p += sprintf(trans->ivy_p, "%u", (uint32_t)(*((uint32_t *)(b + i))));
        i += 4;
        break;
      case DL_TYPE_UINT64:
#if __WORDSIZE == 64
        trans->ivy_p += sprintf(trans->ivy_p, "%lu", (uint64_t)(*((uint64_t *)(b + i))));
#else
        trans->ivy_p += sprintf(trans->ivy_p, "%llu", (uint64_t)(*((uint64_t *)(b + i))));
#endif
        i += 8;
        break;
      case DL_TYPE_INT8:
        trans->ivy_p += sprintf(trans->ivy_p, "%d", (int8_t)(*((int8_t *)(b + i))));
        i++;
        break;
      case DL_TYPE_INT16:
        trans->ivy_p += sprintf(trans->ivy_p, "%d", (int16_t)(*((int16_t *)(b + i))));
        i += 2;
        break;
      case DL_TYPE_INT32:
        trans->ivy_p += sprintf(trans->ivy_p, "%d", (int32_t)(*((int32_t *)(b + i))));
        i += 4;
        break;
      case DL_TYPE_INT64:
#if __WORDSIZE == 64
        trans->ivy_p += sprintf(trans->ivy_p, "%ld", (uint64_t)(*((uint64_t *)(b + i))));
#else
        trans->ivy_p += sprintf(trans->ivy_p, "%lld", (uint64_t)(*((uint64_t *)(b + i))));
#endif
        i += 8;
        break;
      case DL_TYPE_FLOAT:
        trans->ivy_p += sprintf(trans->ivy_p, "%f", (float)(*((float *)(b + i))));
        i += 4;
        break;
      case DL_TYPE_DOUBLE:
        trans->ivy_p += sprintf(trans->ivy_p, "%f", (double)(*((double *)(b + i))));
        i += 8;
        break;
      case DL_TYPE_ARRAY_LENGTH:
      default:
        // Don't print array length but increment index
        i++;
        break;
    }
    // Coma delimiter for array, no delimiter for char array (string), space otherwise
    if (format == DL_FORMAT_ARRAY) {
      if (type != DL_TYPE_CHAR) {
        trans->ivy_p += sprintf(trans->ivy_p, ",");
      }
    } else {
      trans->ivy_p += sprintf(trans->ivy_p, " ");
    }
  }

  // space end delimiter for arrays, additionally un-quote char arrays (strings)
  if (format == DL_FORMAT_ARRAY) {
    if (type == DL_TYPE_CHAR) {
      trans->ivy_p += sprintf(trans->ivy_p, "\" ");
    } else {
      trans->ivy_p += sprintf(trans->ivy_p, " ");
    }
  }
}

static void put_named_byte(struct ivy_transport *trans, struct link_device *dev __attribute__((unused)),
                           enum TransportDataType type __attribute__((unused)), enum TransportDataFormat format __attribute__((unused)),
                           uint8_t byte __attribute__((unused)), const char *name __attribute__((unused)))
{
  trans->ivy_p += sprintf(trans->ivy_p, "%s ", name);
}

static uint8_t size_of(struct ivy_transport *trans __attribute__((unused)), uint8_t len)
{
  return len;
}

static void start_message(struct ivy_transport *trans, struct link_device *dev __attribute__((unused)),
                          uint8_t payload_len __attribute__((unused)))
{
  trans->ivy_p = trans->ivy_buf;
}

static void end_message(struct ivy_transport *trans, struct link_device *dev __attribute__((unused)))
{
  *(--trans->ivy_p) = '\0';
  if (trans->ivy_dl_enabled) {
    IvySendMsg("%s", trans->ivy_buf);
    downlink.nb_msgs++;
  }
}

static void overrun(struct ivy_transport *trans __attribute__((unused)),
                    struct link_device *dev __attribute__((unused)))
{
  downlink.nb_ovrn++;
}

static void count_bytes(struct ivy_transport *trans __attribute__((unused)),
                        struct link_device *dev __attribute__((unused)), uint8_t bytes)
{
  downlink.nb_bytes += bytes;
}

static int check_available_space(struct ivy_transport *trans __attribute__((unused)),
                                 struct link_device *dev __attribute__((unused)), uint8_t bytes __attribute__((unused)))
{
  return TRUE;
}

static int check_free_space(struct ivy_transport *p __attribute__((unused)), uint8_t len __attribute__((unused))) { return TRUE; }
static void transmit(struct ivy_transport *p __attribute__((unused)), uint8_t byte __attribute__((unused))) {}
static void send_message(struct ivy_transport *p __attribute__((unused))) {}
static int null_function(struct ivy_transport *p __attribute__((unused))) { return 0; }

void ivy_transport_init(void)
{
  ivy_tp.ivy_p = ivy_tp.ivy_buf;
  ivy_tp.ivy_dl_enabled = TRUE;

  ivy_tp.trans_tx.size_of = (size_of_t) size_of;
  ivy_tp.trans_tx.check_available_space = (check_available_space_t) check_available_space;
  ivy_tp.trans_tx.put_bytes = (put_bytes_t) put_bytes;
  ivy_tp.trans_tx.put_named_byte = (put_named_byte_t) put_named_byte;
  ivy_tp.trans_tx.start_message = (start_message_t) start_message;
  ivy_tp.trans_tx.end_message = (end_message_t) end_message;
  ivy_tp.trans_tx.overrun = (overrun_t) overrun;
  ivy_tp.trans_tx.count_bytes = (count_bytes_t) count_bytes;
  ivy_tp.trans_tx.impl = (void *)(&ivy_tp);
  ivy_tp.device.check_free_space = (check_free_space_t) check_free_space;
  ivy_tp.device.put_byte = (put_byte_t) transmit;
  ivy_tp.device.send_message = (send_message_t) send_message;
  ivy_tp.device.char_available = (char_available_t) null_function;
  ivy_tp.device.get_byte = (get_byte_t) null_function;
  ivy_tp.device.periph = (void *)(&ivy_tp);
}
