/*
 * Copyright (C) 2009-2010 Paparazzi Team
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
 */

#include <stdint.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>


#include "fms/fms_debug.h"
#include "fms/fms_spi_link.h"
#include "fms/fms_autopilot_msg.h"

#define fill_msg fill_msg_random
static void fill_msg_counter(struct AutopilotMessageCRCFrame *msg);
static void fill_msg_cst(struct AutopilotMessageCRCFrame *msg);
static void fill_msg_random(struct AutopilotMessageCRCFrame *msg);
static void print_up_msg(struct AutopilotMessageCRCFrame *msg);
static void print_down_msg(struct AutopilotMessageCRCFrame *msg);


int main(int argc, char *argv[])
{

  uint32_t us_delay;

  if (argc > 1) {
    us_delay = atoi(argv[1]);
  } else {
    us_delay = 1953;
  }

  printf("Delay: %dus\n", us_delay);

  if (spi_link_init()) {
    TRACE(TRACE_ERROR, "%s", "failed to open SPI link \n");
    return -1;
  }

  uint8_t skip_buf_check = 0;
  uint8_t skip_crc_check = 0;

  uint32_t buf_check_errors = 0;

  while (1) {
    struct AutopilotMessageCRCFrame crc_msg_out;
    struct AutopilotMessageCRCFrame msg_out_prev;
    struct AutopilotMessageCRCFrame crc_msg_in;
    uint8_t crc_valid;

    /* backup message for later comparison */
    memcpy(&msg_out_prev, &crc_msg_out, sizeof(struct AutopilotMessageCRCFrame));
    /* fill message with data */
    fill_msg(&crc_msg_out);
    /* send it over spi */
    spi_link_send(&crc_msg_out, sizeof(struct AutopilotMessageCRCFrame), &crc_msg_in, &crc_valid);

    /* check that received message is identical to the one previously sent */
    if (!skip_buf_check && spi_link.msg_cnt > 1) {
      if (memcmp(&crc_msg_in.payload, &msg_out_prev.payload, sizeof(struct OVERO_LINK_MSG_DOWN))) {
        printf("Compare failed: (received != expected): \n");
        print_up_msg(&crc_msg_in);
        print_down_msg(&msg_out_prev);
        buf_check_errors++;
      }
    }
    /* report crc error */
    if (!skip_crc_check & !crc_valid) {
      printf("CRC checksum failed: received %04X != computed %04X\n",
             crc_msg_in.crc,
             crc_calc_block_crc8((uint8_t *)&crc_msg_in.payload, sizeof(struct OVERO_LINK_MSG_DOWN)));
    }
    /* report message count */
    if (!(spi_link.msg_cnt % 1000))
      printf("msg %d, buf err %d, CRC errors: %d\n", spi_link.msg_cnt,
             buf_check_errors, spi_link.crc_err_cnt);

    /* give it some rest */
    if (us_delay > 0) {
      usleep(us_delay);
    }
  }

  return 0;
}


static void print_up_msg(struct AutopilotMessageCRCFrame *msg)
{
  printf("UP: %08X %08X %08X %08X %08X %08X %08X %08X CRC: %08X\n",
         msg->payload.msg_up.foo,
         msg->payload.msg_up.bar,
         msg->payload.msg_up.bla,
         msg->payload.msg_up.ble,
         msg->payload.msg_up.bli,
         msg->payload.msg_up.blo,
         msg->payload.msg_up.blu,
         msg->payload.msg_up.bly,
         msg->crc);
}
static void print_down_msg(struct AutopilotMessageCRCFrame *msg)
{
  printf("DW: %08X %08X %08X %08X %08X %08X %08X %08X CRC: %08X\n",
         msg->payload.msg_down.foo,
         msg->payload.msg_down.bar,
         msg->payload.msg_down.bla,
         msg->payload.msg_down.ble,
         msg->payload.msg_down.bli,
         msg->payload.msg_down.blo,
         msg->payload.msg_up.blu,
         msg->payload.msg_up.bly,
         msg->crc);
}


static void fill_msg_counter(struct AutopilotMessageCRCFrame *msg)
{
  static uint32_t foo = 5000;
  msg->payload.msg_up.foo = 0x55;
  msg->payload.msg_up.bar = 1;
  msg->payload.msg_up.bla = 0xff;
  msg->payload.msg_up.ble = foo % 255;
  msg->payload.msg_up.bli = 1;
  msg->payload.msg_up.blo = 0xff;
  msg->payload.msg_up.blu = 0;
  msg->payload.msg_up.bly = 0;

  foo--;
  if (foo == 0) {
    foo = 5000;
  }
}

static void fill_msg_cst(struct AutopilotMessageCRCFrame *msg)
{
  msg->payload.msg_up.foo = 0;
  msg->payload.msg_up.bar = 0;
  msg->payload.msg_up.bla = 0;
  msg->payload.msg_up.ble = 0;
  msg->payload.msg_up.bli = 0;
  msg->payload.msg_up.blo = 0;
  msg->payload.msg_up.blu = 0;
  msg->payload.msg_up.bly = 0x01;
}

static void fill_msg_random(struct AutopilotMessageCRCFrame *msg)
{
  msg->payload.msg_up.foo = random();
  msg->payload.msg_up.bar = random();
  msg->payload.msg_up.bla = random();
  msg->payload.msg_up.ble = random();
  msg->payload.msg_up.bli = random();
  msg->payload.msg_up.blo = random();
  msg->payload.msg_up.blu = random();
  msg->payload.msg_up.bly = random();
}
