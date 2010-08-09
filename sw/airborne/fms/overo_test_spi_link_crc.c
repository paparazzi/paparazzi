/*
 * $Id$
 *
 * Copyright (C) 2008-2009 Antoine Drouin <poinix@gmail.com>
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

#include "fms/fms_crc.h"

static void fill_msg(struct AutopilotMessageCRCFrame * msg); 
static void print_up_msg(struct AutopilotMessageCRCFrame * msg);
static void print_down_msg(struct AutopilotMessageCRCFrame * msg);

static uint32_t passed_cnt = 0;
static uint32_t crc_errors = 0;

int main(int argc, char *argv[]) {
  
  unsigned char skip_test; 
  uint32_t us_delay; 
  
  if(argc > 1) { 
    us_delay = atoi(argv[1]);
  }
  else { 
    us_delay = 1953; 
  }

  printf("Delay: %dus\n", us_delay); 
  
  //  crc__init(0x31); 
  crc__init(0x07); 
  printf("CRC initialized\n");

  if (spi_link_init()) {
    TRACE(TRACE_ERROR, "%s", "failed to open SPI link \n");
    return -1;
  }

  skip_test = 1; 
  while (1) {
    struct AutopilotMessageCRCFrame crc_msg_out;
    struct AutopilotMessageCRCFrame msg_out_prev;
    struct AutopilotMessageCRCFrame crc_msg_in;
    crc_t crc_in;
    
    memcpy(&msg_out_prev, &crc_msg_out, sizeof(struct AutopilotMessageCRCFrame));
    
    fill_msg(&crc_msg_out);
    
    crc_msg_out.crc = crc__calc_block_crc8(&crc_msg_out, sizeof(struct OVERO_LINK_MSG_DOWN));
    spi_link_send(&crc_msg_out, sizeof(struct AutopilotMessageCRCFrame), &crc_msg_in);
    crc_in          = crc__calc_block_crc8(&crc_msg_in, sizeof(struct OVERO_LINK_MSG_DOWN));
    
    if(crc_msg_in.crc != crc_in) {
      crc_errors++; 
      printf("CRC checksum failed: received %04X != computed %04X (out: %04X)\n", 
              crc_msg_in.crc, crc_in, crc_msg_out.crc);
    }

    if (!skip_test && memcmp(&crc_msg_in.payload, &msg_out_prev.payload, sizeof(struct OVERO_LINK_MSG_DOWN))) {
      printf("Compare failed: (received != expected): \n");
      print_up_msg(&crc_msg_in);
      print_down_msg(&msg_out_prev);
//    printf("! expected %d %d %d %d\n", msg_out_prev.payload.msg_down.foo, msg_out_prev.payload.msg_down.bar, msg_out_prev.payload.msg_down.bla, msg_out_prev.payload.msg_down.ble);
//    printf("! got      %d %d %d %d\n\n", crc_msg_in.payload.msg_up.foo, crc_msg_in.payload.msg_up.bar, crc_msg_in.payload.msg_up.bla, crc_msg_in.payload.msg_up.ble);
    }
    else {
      if(crc_msg_in.crc != crc_in) { 
        printf("False CRC error\n");
      }
      passed_cnt++;
      skip_test = 0; 
      if (!(passed_cnt % 1000)) {
        printf("passed %d, CRC errors: %d\n", passed_cnt, crc_errors);
      }
    }
    if(us_delay > 0) { 
      usleep(us_delay);
    }
  }

  return 0;
}


static void print_up_msg(struct AutopilotMessageCRCFrame * msg) {
  printf("UP: %08X %08X %08X %08X %08X %08X %08X %08X CRC: %08X\n", msg->payload.msg_up.foo, 
                                              msg->payload.msg_up.bar, 
                                              msg->payload.msg_up.bla, 
                                              msg->payload.msg_up.ble, 
                                              msg->payload.msg_up.bli, 
                                              msg->payload.msg_up.blo, 
                                              msg->payload.msg_up.blu, 
                                              msg->payload.msg_up.bly, 
                                              msg->crc);
}
static void print_down_msg(struct AutopilotMessageCRCFrame * msg) {
  printf("DW: %08X %08X %08X %08X %08X %08X %08X %08X CRC: %08X\n", msg->payload.msg_down.foo, 
                                        msg->payload.msg_down.bar, 
                                        msg->payload.msg_down.bla, 
                                        msg->payload.msg_down.ble, 
                                        msg->payload.msg_down.bli, 
                                        msg->payload.msg_down.blo, 
                                        msg->payload.msg_up.blu, 
                                        msg->payload.msg_up.bly, 
                                        msg->crc);
}


#if 1
static void fill_msg(struct AutopilotMessageCRCFrame * msg) {
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
  if(foo == 0) { 
    foo = 5000;
  }
}
#else
static void fill_msg(struct AutopilotMessageCRCFrame * msg) {
  msg->payload.msg_up.foo = 0;
  msg->payload.msg_up.bar = 0;
  msg->payload.msg_up.bla = 0;
  msg->payload.msg_up.ble = 0;
  msg->payload.msg_up.bli = 0;
  msg->payload.msg_up.blo = 0;
  msg->payload.msg_up.blu = 0;
  msg->payload.msg_up.bly = 0x01;
}
#endif
