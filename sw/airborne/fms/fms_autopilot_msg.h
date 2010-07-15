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
  uint8_t foo;
  uint8_t bar;
  uint8_t blaa;
  uint8_t bli;
};

union AutopilotMessageFoo1
{
  struct AutopilotMessageFoo up;
  struct AutopilotMessageFoo down;
};


/*
 * BETH
 */
struct __attribute__ ((packed)) AutopilotMessageBethUp
{
  struct Int16Vect3 gyro;
  struct Int16Vect3 accel;
  struct Int16Vect3 bench_sensor;

};

struct __attribute__ ((packed)) AutopilotMessageBethDown
{
  uint8_t motor_front;
  uint8_t motor_back;
};

union AutopilotMessageBeth
{
  struct AutopilotMessageBethUp up;
  struct AutopilotMessageBethDown down;
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

union AutopilotMessageTW
{
  struct AutopilotMessageTWUp up;
  struct AutopilotMessageTWDown down;
};

/*
 * Passthrough, aka biplan
 */
struct __attribute__ ((packed)) AutopilotMessagePTUp
{
  struct Int16Vect3 gyro;
  struct Int16Vect3 accel;
  struct Int16Vect3 mag;
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
};

struct __attribute__ ((packed)) AutopilotMessagePTDown
{
  uint16_t pwm_outputs_usecs[LISA_PWM_NB];
};

union AutopilotMessagePT
{
  struct AutopilotMessagePTUp up;
  struct AutopilotMessagePTDown down;
};

#endif /* FMS_AUTOPILOT_H */
