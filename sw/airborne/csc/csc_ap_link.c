#include <string.h>
#include "csc_ap_link.h"

struct CscServoCmd csc_servo_cmd;
struct CscMotorMsg csc_motor_msg;
int32_t csc_ap_link_error_cnt;


void csc_ap_link_init(void) {

  csc_ap_link_error_cnt = 0;

}

void csc_ap_send_msg(uint8_t msg_id, const uint8_t *buf, uint8_t len)
{
  struct CscCanMsg out_msg;

  if (len > 8) return;

  // set frame length
  out_msg.frame = (len << 16);
  // build msg id using our board ID and requested msg id
  out_msg.id = ((CSC_BOARD_ID << 7) & CSC_BOARD_MASK) | (msg_id & CSC_MSGID_MASK);
  // copy msg payload in host order
  memcpy((char *)&out_msg.dat_a, buf, len);
  // send via CAN2 
  csc_can2_send(&out_msg);
}

