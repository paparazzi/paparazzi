/*
 * Copyright (C) 2012 Piotr Esden-Tempski <piotr@esden.net>
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

#include <stdint.h>

#include "mcu_periph/can.h"
#include "mcu_periph/can_arch.h"

#if USE_CAN1

struct can_periph can1 = {
  .fd = 1,
  .callbacks = {0},
  .callback_user_data = {0}
};
#endif

static const uint8_t dlc_to_len[] = {0,1,2,3,4,5,6,7,8,12,16,20,24,32,48,64};

uint8_t can_dlc_to_len(uint8_t dlc) {
  if(dlc < 16) {
    return dlc_to_len[dlc];
  }
  return 0;
}

uint8_t can_len_to_dlc(uint8_t len) {
  if(len <= 8) {
    return len;
  }
  for(int i=9; i<16; i++) {
    if(dlc_to_len[i] >= len) {
      return i;
    }
  }
  return 15;
}


void can_init()
{
  can_hw_init();
}


static int add_can_callback(struct can_periph* canp, can_rx_frame_callback_t callback, void* user_data) {
  for(int i =0; i<CAN_NB_CALLBACKS_MAX; i++) {
    // use the first available slot
    if(canp->callbacks[i] == NULL) {
      canp->callbacks[i] = callback;
      canp->callback_user_data[i] = user_data;
      return 0;
    }
  }
  // no available slot
  return -1;
}

int can_register_callback(can_rx_frame_callback_t callback, struct pprzaddr_can* src_addr, void* user_data) {
  #if USE_CAN1
  if(src_addr->can_ifindex == 1 || src_addr->can_ifindex == 0) {
    int ret = add_can_callback(&can1, callback, user_data);
    if(ret) { return ret; }
  }
  #endif
  #if USE_CAN2
  if(src_addr->can_ifindex == 2 || src_addr->can_ifindex == 0) {
    int ret = add_can_callback(&can2, callback, user_data);
    if(ret) { return ret; }
  }
  #endif
  return 0;
}
