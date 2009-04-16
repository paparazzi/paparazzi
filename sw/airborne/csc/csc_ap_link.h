#ifndef CSC_AP_LINK_H
#define CSC_AP_LINK_H

#include "std.h"

#define CSC_SERVO_CMD_ID    0
#define CSC_MOTOR_CMD_ID    1
#define CSC_MOTOR_STATUS_ID 2


/* Received from the autopilot */
struct CscServoCmd {
  uint16_t servo_1;
  uint16_t servo_2;
  uint16_t servo_3;
  uint16_t servo_4;
};

struct CscMotorCmd {
  uint8_t cmd_id;
  uint16_t arg1;
  uint16_t arg2;

};

/* Sent to the autopilot */
struct CscMotorStatus {
uint16_t uart_throttle_faultflags;
uint16_t uart_throttle_rpm;
uint16_t uart_throttle_vbus;

};


extern struct CscServoCmd csc_servo_cmd;
extern struct CscMotorCmd csc_motor_cmd;
extern void csc_ap_link_init(void);


#include "csc_can.h"

#define CSC_AP_LINK_ON_CAN_MSG(_servo_handler, _motor_handler) {	\
									\
  }


#endif /* CSC_AP_LINK_H */

