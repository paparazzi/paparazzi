#include <string.h>
#include "csc_ap_link.h"
#include "csc_can.h"
#include "led.h"


int32_t csc_ap_link_error_cnt;

static void (* servo_msg_cb)(struct CscServoCmd *);
static void (* motor_msg_cb)(struct CscMotorMsg *);
static void (* rc_msg_cb)(struct CscRCMsg *);
static void (* prop_msg_cb)(struct CscPropCmd *, int idx);
static void (* gpsfix_cb)(struct CscGPSFixMsg *);
static void (* gpspos_cb)(struct CscGPSPosMsg *);
static void (* gpsacc_cb)(struct CscGPSAccMsg *);

void csc_ap_link_send_adc(float adc1, float adc2)
{
  struct CscADCMsg msg;

  msg.ADCVolts1 = adc1;
  msg.ADCVolts2 = adc2;

  csc_ap_send_msg(CSC_BOARD_ADCVOLTS_ID, (const uint8_t *) &msg, sizeof(msg));
}

void csc_ap_link_send_status(uint32_t loops, uint32_t msgs)
{

  struct CscStatusMsg msg;

  msg.loop_count = loops;
  msg.msg_count = msgs;

  csc_ap_send_msg(CSC_BOARD_STATUS_ID, (const uint8_t *) &msg, sizeof(msg));
}

void csc_ap_link_send_vane(float vane_angle)
{

  struct CscVaneMsg msg;

  msg.vane_angle1 = vane_angle;

  csc_ap_send_msg(CSC_VANE_MSG_ID, (const uint8_t *) &msg, sizeof(msg));
}


// Generic function for sending can messages
void can_write_csc(uint8_t board_id, uint8_t msg_id, const uint8_t *buf, uint8_t len)
{
  struct CscCanMsg out_msg;

  if (len > 8) return;

  // set frame length
  out_msg.frame = (len << 16);
  // build msg id using our board ID and requested msg id
  out_msg.id = ((board_id & CSC_BOARD_MASK) << 7) | (msg_id & CSC_MSGID_MASK);
  // copy msg payload in host order
  memcpy((char *)&out_msg.dat_a, buf, len);
  // send via CAN 
  csc_can1_send(&out_msg);
}

void csc_ap_send_msg(uint8_t msg_id, const uint8_t *buf, uint8_t len)
{
  can_write_csc(CSC_BOARD_ID, msg_id, buf, len);
}

void csc_ap_link_set_servo_cmd_cb(void (* cb)(struct CscServoCmd *cmd))
{
  servo_msg_cb = cb;
}

void csc_ap_link_set_motor_cmd_cb(void (* cb)(struct CscMotorMsg *msg))
{
  motor_msg_cb = cb;
}

void csc_ap_link_set_prop_cmd_cb(void (* cb)(struct CscPropCmd *cmd, int idx))
{
  prop_msg_cb = cb;
}

void csc_ap_link_set_rc_cmd_cb(void (* cb)(struct CscRCMsg *msg))
{
  rc_msg_cb = cb;
}

void csc_ap_link_set_gpsfix_cb(void (* cb)(struct CscGPSFixMsg *msg))
{
  gpsfix_cb = cb;
}

void csc_ap_link_set_gpspos_cb(void (* cb)(struct CscGPSPosMsg *msg))
{
  gpspos_cb = cb;
}

void csc_ap_link_set_gpsacc_cb(void (* cb)(struct CscGPSAccMsg *msg))
{
  gpsacc_cb = cb;
}

static void on_ap_link_msg( struct CscCanMsg *msg)
{
  uint32_t msg_id = MSGID_OF_CANMSG_ID(msg->id);
  uint32_t len = CAN_MSG_LENGTH_OF_FRAME(msg->frame);
  switch (msg_id) {
    case CSC_SERVO_CMD_ID:
      if (len != sizeof(struct CscServoCmd)) {
	LED_ON(ERROR_LED);
	csc_ap_link_error_cnt++;
      } else {
	// cast can data buffer pointer directly to csc_servo_cmd pointer
	servo_msg_cb((struct CscServoCmd *) &msg->dat_a);
      }
      break;
    case CSC_MOTOR_CMD_ID:
      if (len != sizeof(struct CscMotorMsg)) {
	LED_ON(ERROR_LED);
	csc_ap_link_error_cnt++;
      } else {
	// cast can data buffer pointer directly to csc_motor_msg pointer
	//motor_msg_cb((struct CscMotorMsg *) &msg->dat_a);
	LED_TOGGLE(ERROR_LED);
      }
      break;
    case CSC_RC_ID:
      if (len != sizeof(struct CscRCMsg)) {
	LED_ON(ERROR_LED);
	csc_ap_link_error_cnt++;
      } else {
	// cast can data buffer pointer directly to csc_motor_msg pointer
	rc_msg_cb((struct CscRCMsg *) &msg->dat_a);
      }
      break;
    case CSC_PROP_CMD_ID:
      if (len != sizeof(struct CscPropCmd)){
	LED_ON(ERROR_LED);
	csc_ap_link_error_cnt++;
      } else
	prop_msg_cb((struct CscPropCmd *) &msg->dat_a, 0);
      break;
    case CSC_GPS_FIX_ID:
      if (len != sizeof(struct CscGPSFixMsg)){
	LED_ON(ERROR_LED);
	csc_ap_link_error_cnt++;
      } else
	gpsfix_cb((struct CscGPSFixMsg *) &msg->dat_a);
      break;
    case CSC_GPS_POS_ID:
      if (len != sizeof(struct CscGPSPosMsg)){
	LED_ON(ERROR_LED);
	csc_ap_link_error_cnt++;
      } else
	gpspos_cb((struct CscGPSPosMsg *) &msg->dat_a);
      break;
    case CSC_GPS_ACC_ID:
      if (len != sizeof(struct CscGPSAccMsg)){
	LED_ON(ERROR_LED);
	csc_ap_link_error_cnt++;
      } else
	gpsacc_cb((struct CscGPSAccMsg *) &msg->dat_a);
      break;
    case CSC_PROP2_CMD_ID:
      if (len != sizeof(struct CscPropCmd)){
	LED_ON(ERROR_LED);
	csc_ap_link_error_cnt++;
      } else
	prop_msg_cb((struct CscPropCmd *) &msg->dat_a, 1);
      break;
  }
}

void csc_ap_link_init(void)
{
  csc_can1_init(on_ap_link_msg);
  csc_ap_link_error_cnt = 0;
}

