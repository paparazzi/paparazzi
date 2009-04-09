#ifndef CSC_CAN_H
#define CSC_CAN_H

#include <inttypes.h>

// Common CAN bit rates
#define   CANBitrate125k_12MHz          0x001C001D
#define   CANBitrate250k_12MHz          0x001C000E

struct CscCanMsg {
  uint32_t frame;  // Bits 16..19: DLC - Data Length Counter
                   // Bit 30: Set if this is a RTR message
                   // Bit 31: Set if this is a 29-bit ID message
  uint32_t msg_id; // CAN Message ID (11-bit or 29-bit)
  uint32_t dat_a;  // CAN Message Data Bytes 0-3
  uint32_t dat_b;  // CAN Message Data Bytes 4-7
};


extern void csc_can1_init(void);
extern void csc_can1_send(struct CscCanMsg* msg);


extern void csc_can2_init(void);


#endif /* CSC_CAN_H */

