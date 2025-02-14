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

#ifndef CAN_H
#define CAN_H

#include "std.h"
#include "mcu_periph/can_arch.h"

#ifndef CAN_FD_MODE
#define CAN_FD_MODE TRUE
#endif

#ifdef CAN_FD_MODE
#define SOCKETCAN_MAX_DLEN 64U
#else
#define SOCKETCAN_MAX_DLEN 8U
#endif

#ifndef CAN_NB_CALLBACKS_MAX
#define CAN_NB_CALLBACKS_MAX 10
#endif

// CAN identifier
// +------+--------------------------------------------------------------+
// | Bits | Description                                                  |
// +======+==============================================================+
// | 0-28 | CAN identifier (11/29 bit)                                   |
// +------+--------------------------------------------------------------+
// |  29  | Error message frame flag (0 = data frame, 1 = error message) |
// +------+--------------------------------------------------------------+
// |  30  | Remote transmission request flag (1 = RTR frame)             |
// +------+--------------------------------------------------------------+
// |  31  | Frame format flag (0 = standard 11 bit, 1 = extended 29 bit) |
// +------+--------------------------------------------------------------+
typedef uint32_t socketcan_id_t;

// error flag
#define 	CAN_FRAME_ERR   (1<<29)
// remote transmition request
#define 	CAN_FRAME_RTR   (1<<30)
// extended identifier
#define 	CAN_FRAME_EFF   (1<<31)

#define CAN_EID_MASK 0x1FFFFFFF
#define CAN_SID_MASK 0x7FF

// /* CAN FD specific flags from Linux Kernel (include/uapi/linux/can.h) */
#define CANFD_BRS 0x01 
#define CANFD_ESI 0x02 
#define CANFD_FDF 0x04 

struct pprzcan_frame {
  socketcan_id_t can_id;
  uint8_t len;
  uint8_t flags;  // CAN FD specific flags
  uint32_t timestamp;   // timestamp in ms.
  uint8_t data[SOCKETCAN_MAX_DLEN];
};

// socketaddr_can paparazzi abstraction
struct pprzaddr_can {
        //sa_family_t can_family;   
        int         can_ifindex;  // network interface index
};

typedef void(* can_rx_frame_callback_t)(struct pprzcan_frame* rxframe, struct pprzaddr_can* src_addr, void* user_data);


struct can_periph {
  void* arch_struct;
  int fd;
  can_rx_frame_callback_t callbacks[CAN_NB_CALLBACKS_MAX];
  void* callback_user_data[CAN_NB_CALLBACKS_MAX];
};

#if USE_CAN1
extern struct can_periph can1;
#endif
#if USE_CAN2
extern struct can_periph can2;
#endif

void can_init(void);

/**
 * Add a callback on received frames from an interface
 * @param callback The callback called on received frames
 * @param src_addr Interface from which frames are received. 0 means all interfaces.
 * @param user_data Pointer that will be passed in callback parameters
 * @return 0 if the callback was successfully added.
 */
int can_register_callback(can_rx_frame_callback_t callback, struct pprzaddr_can* src_addr, void* user_data);

int can_transmit_frame(struct pprzcan_frame* txframe, struct pprzaddr_can* dst_addr);


uint8_t can_dlc_to_len(uint8_t dlc);
uint8_t can_len_to_dlc(uint8_t len);
#endif /* CAN_H */
