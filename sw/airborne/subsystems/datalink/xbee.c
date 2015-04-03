/*
 * Copyright (C) 2006  Pascal Brisset, Antoine Drouin
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
 * @file subsystems/datalink/xbee.c
 * Maxstream XBee serial input and output
 */

#include "mcu_periph/sys_time.h"
#include "subsystems/datalink/uart_print.h"
#include "subsystems/datalink/xbee.h"
#include "subsystems/datalink/downlink.h"

/** Ground station address */
#define GROUND_STATION_ADDR 0x100
/** Aircraft address */
#define XBEE_MY_ADDR AC_ID

/** Constants for the API protocol */
#define TX_OPTIONS 0x00
#define NO_FRAME_ID 0
#define XBEE_API_OVERHEAD 5 /* start + len_msb + len_lsb + API_id + checksum */

struct xbee_transport xbee_tp;

#define AT_COMMAND_SEQUENCE "+++"
#define AT_SET_MY "ATMY"
#define AT_AP_MODE "ATAP1\r"
#define AT_EXIT "ATCN\r"

/** Xbee protocol implementation */

static void put_1byte(struct xbee_transport *trans, struct link_device *dev, const uint8_t byte)
{
  trans->cs_tx += byte;
  dev->put_byte(dev->periph, byte);
}

static void put_bytes(struct xbee_transport *trans, struct link_device *dev,
                      enum TransportDataType type __attribute__((unused)), enum TransportDataFormat format __attribute__((unused)),
                      uint8_t len, const void *bytes)
{
  const uint8_t *b = (const uint8_t *) bytes;
  int i;
  for (i = 0; i < len; i++) {
    put_1byte(trans, dev, b[i]);
  }
}

static void put_named_byte(struct xbee_transport *trans, struct link_device *dev,
                           enum TransportDataType type __attribute__((unused)), enum TransportDataFormat format __attribute__((unused)),
                           uint8_t byte, const char *name __attribute__((unused)))
{
  put_1byte(trans, dev, byte);
}

static uint8_t size_of(struct xbee_transport *trans __attribute__((unused)), uint8_t len)
{
  // message length: payload + API overhead + XBEE TX overhead (868 or 2.4)
  return len + XBEE_API_OVERHEAD + XBEE_TX_OVERHEAD;
}

static void start_message(struct xbee_transport *trans, struct link_device *dev, uint8_t payload_len)
{
  downlink.nb_msgs++;
  dev->put_byte(dev->periph, XBEE_START);
  const uint16_t len = payload_len + XBEE_API_OVERHEAD;
  dev->put_byte(dev->periph, (len >> 8));
  dev->put_byte(dev->periph, (len & 0xff));
  trans->cs_tx = 0;
  const uint8_t header[] = XBEE_TX_HEADER;
  put_bytes(trans, dev, DL_TYPE_UINT8, DL_FORMAT_SCALAR, XBEE_TX_OVERHEAD + 1, header);
}

static void end_message(struct xbee_transport *trans, struct link_device *dev)
{
  trans->cs_tx = 0xff - trans->cs_tx;
  dev->put_byte(dev->periph, trans->cs_tx);
  dev->send_message(dev->periph);
}

static void overrun(struct xbee_transport *trans __attribute__((unused)),
                    struct link_device *dev __attribute__((unused)))
{
  downlink.nb_ovrn++;
}

static void count_bytes(struct xbee_transport *trans __attribute__((unused)),
                        struct link_device *dev __attribute__((unused)), uint8_t bytes)
{
  downlink.nb_bytes += bytes;
}

static int check_available_space(struct xbee_transport *trans __attribute__((unused)), struct link_device *dev,
                                 uint8_t bytes)
{
  return dev->check_free_space(dev->periph, bytes);
}

static uint8_t xbee_text_reply_is_ok(struct link_device *dev)
{
  char c[2];
  int count = 0;

  while (dev->char_available(dev->periph)) {
    char cc = dev->get_byte(dev->periph);
    if (count < 2) {
      c[count] = cc;
    }
    count++;
  }

  if ((count > 2) && (c[0] == 'O') && (c[1] == 'K')) {
    return TRUE;
  }

  return FALSE;
}

static uint8_t xbee_try_to_enter_api(struct link_device *dev)
{

  /** Switching to AT mode (FIXME: busy waiting) */
  print_string(dev, AT_COMMAND_SEQUENCE);

  /** - busy wait 1.25s */
  sys_time_usleep(1250000);

  return xbee_text_reply_is_ok(dev);
}


#if XBEE_BAUD == B9600
#define XBEE_BAUD_ALTERNATE B57600
#define XBEE_ATBD_CODE "ATBD3\rATWR\r"
#pragma message "Experimental: XBEE-API@9k6 auto-baudrate 57k6 -> 9k6 (stop ground link for correct operation)"
#elif XBEE_BAUD == B57600
#define XBEE_BAUD_ALTERNATE B9600
#define XBEE_ATBD_CODE "ATBD6\rATWR\r"
#pragma message "Experimental: XBEE-API@57k6 auto-baudrate 9k6 -> 57k6 (stop ground link for correct operation)"
#else
#warning XBEE-API Non default baudrate: auto-baud disabled
#endif


void xbee_init(void)
{
  xbee_tp.status = XBEE_UNINIT;
  xbee_tp.trans_rx.msg_received = FALSE;
  xbee_tp.trans_tx.size_of = (size_of_t) size_of;
  xbee_tp.trans_tx.check_available_space = (check_available_space_t) check_available_space;
  xbee_tp.trans_tx.put_bytes = (put_bytes_t) put_bytes;
  xbee_tp.trans_tx.put_named_byte = (put_named_byte_t) put_named_byte;
  xbee_tp.trans_tx.start_message = (start_message_t) start_message;
  xbee_tp.trans_tx.end_message = (end_message_t) end_message;
  xbee_tp.trans_tx.overrun = (overrun_t) overrun;
  xbee_tp.trans_tx.count_bytes = (count_bytes_t) count_bytes;
  xbee_tp.trans_tx.impl = (void *)(&xbee_tp);

  struct link_device *dev = &((XBEE_UART).device);

  // Empty buffer before init process
  while (dev->char_available(dev->periph)) {
    dev->get_byte(dev->periph);
  }

#ifndef NO_XBEE_API_INIT
  /** - busy wait 1.25s */
  sys_time_usleep(1250000);

  if (! xbee_try_to_enter_api(dev)) {
#ifdef XBEE_BAUD_ALTERNATE

    // Badly configured... try the alternate baudrate:
    uart_periph_set_baudrate(&(XBEE_UART), XBEE_BAUD_ALTERNATE); // FIXME add set_baudrate to generic device, assuming uart for now
    if (xbee_try_to_enter_api(dev)) {
      // The alternate baudrate worked,
      print_string(dev, XBEE_ATBD_CODE);
    } else {
      // Complete failure, none of the 2 baudrates result in any reply
      // TODO: set LED?

      // Set the default baudrate, just in case everything is right
      uart_periph_set_baudrate(&(XBEE_UART), XBEE_BAUD); // FIXME add set_baudrate to generic device, assuming uart for now
      print_string(dev, "\r");
    }

#endif
    // Continue changing settings until the EXIT is issued.
  }

  /** Setting my address */
  print_string(dev, AT_SET_MY);
  uint16_t addr = XBEE_MY_ADDR;
  print_hex16(dev, addr);
  print_string(dev, "\r");

  print_string(dev, AT_AP_MODE);

#ifdef XBEE_INIT
  print_string(dev, XBEE_INIT);
#endif

  /** Switching back to normal mode */
  print_string(dev, AT_EXIT);

  uart_periph_set_baudrate(&(XBEE_UART), XBEE_BAUD); // FIXME add set_baudrate to generic device, assuming uart for now

#endif
}
