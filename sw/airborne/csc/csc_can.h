#ifndef CSC_CAN_H
#define CSC_CAN_H

#include <inttypes.h>
#include "std.h"


#define BOARDID_OF_CANMSG_ID(_id) (((_id)>>7) & CSC_BOARD_MASK)
#define MSGID_OF_CANMSG_ID(_id) ((_id) & CSC_MSGID_MASK)
#define CAN_MSG_LENGTH_OF_FRAME(_f) (((_f)>>16) & 0x0F)

// Common CAN bit rates
#define   CANBitrate62k5_1MHz           0x001C001D
#define   CANBitrate125k_2MHz           0x001C000E

#define CSC_BOARD_MASK 0x0F
#define CSC_MSGID_MASK 0x7F

struct CscCanMsg {
  uint32_t frame;  // Bits 16..19: DLC - Data Length Counter
                   // Bit 30: Set if this is a RTR message
                   // Bit 31: Set if this is a 29-bit ID message
  uint32_t id;     // CAN Message ID (11-bit or 29-bit)
  uint32_t dat_a;  // CAN Message Data Bytes 0-3
  uint32_t dat_b;  // CAN Message Data Bytes 4-7
};

void csc_can_event(void);

#ifdef USE_CAN1
void csc_can1_init(void(* callback)(struct CscCanMsg *msg));
void csc_can1_send(struct CscCanMsg* msg);

extern bool_t can1_msg_received;
extern struct CscCanMsg can1_rx_msg;


#endif /* USE_CAN1 */


#endif /* CSC_CAN_H */

