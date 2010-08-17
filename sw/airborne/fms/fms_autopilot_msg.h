#ifndef FMS_AUTOPILOT_H
#define FMS_AUTOPILOT_H

#include <inttypes.h>
#include "math/pprz_algebra_int.h"
#include "airframe.h"

#define LISA_PWM_OUTPUT_NB 6

/*
 * Testing
 */

struct __attribute__ ((packed)) AutopilotMessageFoo
{
  uint32_t foo;
  uint32_t bar;
  uint32_t bla;
  uint32_t ble;
  uint32_t bli;
  uint32_t blo;
  uint32_t blu;
  uint32_t bly;
};

/*
 * BETH
 */
struct __attribute__ ((packed)) AutopilotMessageBethUp
{
  struct Int16Rates gyro;
  struct Int16Vect3 accel;
  struct Int16Vect3 bench_sensor;
  uint16_t can_errs;
  uint16_t spi_errs;
  uint32_t cnt;
  int8_t thrust_out;
  int8_t pitch_out;
};

struct __attribute__ ((packed)) AutopilotMessageBethDown
{
  uint8_t thrust;
  uint8_t pitch;
  uint32_t errors;
  uint32_t cnt;
};

/*
 *  STM Telemetry through wifi
 */
#define TW_BUF_LEN 63
struct __attribute__ ((packed)) AutopilotMessageTWUp
{
  uint8_t tw_len;
  uint8_t data[TW_BUF_LEN];
};

struct __attribute__ ((packed)) AutopilotMessageTWDown
{
  uint8_t tw_len;
  uint8_t data[TW_BUF_LEN];
};

/*
 * Passthrough, aka biplan
 */

/* used to indicate parts of the message which actually represent a new measurement */
struct __attribute__ ((packed)) PTUpValidFlags
{
  unsigned rc:1;
  unsigned pressure_absolute:1;
  unsigned pressure_differential:1;
  unsigned vane:1;
  unsigned imu:1;
};

struct __attribute__ ((packed)) AutopilotMessagePTUp
{
  struct Int32Rates gyro;
  struct Int16Vect3 accel;
  struct Int16Vect3 mag;
  int16_t pressure_absolute;
  int16_t pressure_differential;
  int16_t rc_pitch;
  int16_t rc_roll;
  int16_t rc_yaw;
  int16_t rc_thrust;
  int16_t rc_mode;
  int16_t rc_kill;
  int16_t rc_gear;
  int16_t rc_aux3;
  int16_t rc_aux4;
  uint8_t rc_status;
  struct PTUpValidFlags valid;
  uint32_t stm_msg_cnt;
  uint32_t stm_crc_err_cnt;
};

struct __attribute__ ((packed)) AutopilotMessagePTDown
{
  uint16_t pwm_outputs_usecs[LISA_PWM_OUTPUT_NB];
};

/* Union for computing size of SPI transfer (largest of either up or down message) */
union AutopilotMessage {
  struct OVERO_LINK_MSG_UP msg_up;
  struct OVERO_LINK_MSG_DOWN msg_down;
};

struct __attribute__ ((packed)) AutopilotMessageCRCFrame
{
  union AutopilotMessage payload; 
  uint8_t crc; 
}; 

#endif /* FMS_AUTOPILOT_H */
