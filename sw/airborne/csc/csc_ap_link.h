#ifndef CSC_AP_LINK_H
#define CSC_AP_LINK_H

#include "std.h"

#define CSC_SERVO_CMD_ID    0
#define CSC_MOTOR_CMD_ID    1
#define CSC_MOTOR_STATUS_ID 2

#include "csc_can.h"

/* Received from the autopilot */
struct CscServoCmd {
  uint16_t servo_1;
  uint16_t servo_2;
  uint16_t servo_3;
  uint16_t servo_4;
};

struct CscMotorCmd {
  uint8_t  cmd_id;
  uint8_t  pad1;
  uint16_t arg1;
  uint16_t arg2;
};

/* Sent to the autopilot */
struct CscMotorStatus {
  uint16_t uart_throttle_faultflags;
  uint16_t uart_throttle_rpm;
  uint16_t uart_throttle_vbus;
};


extern struct CscServoCmd    csc_servo_cmd;
extern struct CscMotorCmd    csc_motor_cmd;
extern struct CscMotorStatus csc_motor_status;

extern void csc_ap_link_init(void);

#define CSC_MSG_MASK 0x7F

#include "csc_can.h"

#define CscApLinkEvent(_on_servo_msg, _on_motor_msg) {		\
    Can2Event(CscApLinkOnCanMsg(_on_servo_msg, _on_motor_msg));	\
  }

#define CscApLinkOnCanMsg(_on_servo_msg, _on_motor_msg) {	  \
    uint32_t msg_id = (can2_rx_msg.id & CSC_MSG_MASK);		  \
    switch (msg_id) {						  \
    case CSC_SERVO_CMD_ID:					  \
      csc_servo_cmd.servo_1 = can2_rx_msg.dat_a&0xFFFF;		  \
      csc_servo_cmd.servo_2 = (can2_rx_msg.dat_a>>16)&0xFFFF;	  \
      csc_servo_cmd.servo_3 = can2_rx_msg.dat_b&0xFFFF;		  \
      csc_servo_cmd.servo_4 = (can2_rx_msg.dat_b>>16)&0xFFFF;	  \
      _on_servo_msg();						  \
      break;							  \
    case CSC_MOTOR_CMD_ID:					  \
      csc_motor_cmd.cmd_id = can2_rx_msg.dat_a & 0xFF;		  \
      csc_motor_cmd.arg1 = (can2_rx_msg.dat_a>>16) & 0xFFFF;	  \
      csc_motor_cmd.arg2 = can2_rx_msg.dat_b & 0xFFFF;		  \
      _on_motor_msg();						  \
      break;							  \
    }								  \
  }


#endif /* CSC_AP_LINK_H */

