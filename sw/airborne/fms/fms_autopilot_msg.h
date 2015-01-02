#ifndef FMS_AUTOPILOT_H
#define FMS_AUTOPILOT_H

#include <inttypes.h>
#include "math/pprz_algebra_int.h"
#include "math/pprz_geodetic_int.h"
// FIXME: why is including airframe.h needed ?
//#include "generated/airframe.h"
//#include "adc.h"
#define NB_ADC 8


#define LISA_PWM_OUTPUT_NB 10

/*
 * Testing
 */

struct __attribute__((packed)) AutopilotMessageFoo {
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
struct __attribute__((packed)) AutopilotMessageBethUp {
  struct Int16Rates gyro;
  struct Int16Vect3 accel;
  struct Int16Vect3 bench_sensor;
  uint16_t can_errs;
  uint16_t spi_errs;
  uint32_t cnt;
  int8_t thrust_out;
  int8_t pitch_out;
};

struct __attribute__((packed)) AutopilotMessageBethDown {
  uint8_t thrust;
  uint8_t pitch;
  uint32_t errors;
  uint32_t cnt;
};

/*
 *  STM Telemetry through wifi
 */
#define TW_BUF_LEN 63
struct __attribute__((packed)) AutopilotMessageTWUp {
  uint8_t tw_len;
  uint8_t data[TW_BUF_LEN];
};

struct __attribute__((packed)) AutopilotMessageTWDown {
  uint8_t tw_len;
  uint8_t data[TW_BUF_LEN];
};

/*
 * Passthrough, aka biplan
 */

struct __attribute__((packed)) ADCMessage {
  uint16_t channels[NB_ADC];
};

/* used to indicate parts of the message which actually represent a new measurement */
struct __attribute__((packed)) PTUpValidFlags {
  unsigned rc: 1;
  unsigned pressure_absolute: 1;
  unsigned pressure_differential: 1;
  unsigned vane: 1;
  unsigned imu: 1;
  unsigned adc: 1;
};

struct __attribute__((packed)) AutopilotMessagePTUp {
  struct Int32Rates gyro;
  struct Int32Vect3 accel;
  struct Int32Vect3 mag;
  uint32_t imu_tick;
  int32_t pressure_absolute;
  int32_t pressure_differential;
  int16_t rc_pitch;
  int16_t rc_roll;
  int16_t rc_yaw;
  int16_t rc_thrust;
  int16_t rc_mode;
  int16_t rc_kill;
  int16_t rc_gear;
  int16_t rc_aux2;
  int16_t rc_aux3;
  uint8_t rc_status;
  float vane_angle1;
  float vane_angle2;
  struct ADCMessage adc;

  struct PTUpValidFlags valid;
  uint32_t stm_msg_cnt;
  uint32_t stm_crc_err_cnt;
};

struct __attribute__((packed)) AutopilotMessagePTDown {
  uint16_t pwm_outputs_usecs[LISA_PWM_OUTPUT_NB];
};



#define VI_IMU_DATA_VALID      0
#define VI_MAG_DATA_VALID      1
#define VI_GPS_DATA_VALID      2
#define VI_BARO_ABS_DATA_VALID 3

struct __attribute__((packed)) AutopilotMessageVIUp {
  struct Int16Rates gyro;
  struct Int16Vect3 accel;
  struct Int16Vect3 mag;
  struct EcefCoor_i ecef_pos;    /* pos ECEF in cm        */
  struct EcefCoor_i ecef_vel;    /* speed ECEF in cm/s    */
  int16_t pressure_absolute;     /* */
  uint8_t valid_sensors;
};

struct __attribute__((packed)) AutopilotMessageVIDown {

};


/*
 * For messages of arbitrary length using fixed DMA
 * buffer size.
 * A message consists of any amount of packages and
 * is recomposed to a raw byte array on application
 * level.
 * Advantage: Interleaving message exchange, constant
 *  latency
 * Disadvantage: Overhead of message / package counters
 *
 * If there is no message to be transferred, an empty
 * package with message_cnt = 0 is sent.
 * The last package of a message has a negative
 * package_cntd that indicates the number of padding
 * (zero) bytes it contains at the end.
 * Example for a 22-byte message transfer with packages
 * of 8 bytes for one side:
 *
 *   message_cnt:   0        message_cnt:  4
 *   package_cntd:  x        package_cntd: 6
 *   data: uint8_t[8]=0      data: uint8_t[8]
 *
 *   ^- invalid package, ignored
 *
 *   message_cnt:   2        message_cnt:  4
 *   package_cntd:  3        package_cnt:  5
 *   data: uint8_t[8]        data: uint8_t[8]
 *
 *   message_cnt:   2        message_cnt:  4
 *   package_cntd:  2        package_cnt:  4
 *   data: uint8_t[8]        data: uint8_t[8]
 *
 *   message_cnt:   2        message_cnt:  4
 *   package_cntd: -5        package_cnt:  3
 *   data: uint8_t[8]        data: uint8_t[8]
 *
 *   --> last package in message, padding in
 *       current message is 5 bytes (-5), so
 *       message length is (3*8)-5 = 22.
 *
 * -- next message
 *
 *   message_cnt:   3        message_cnt:  4
 *   package_cntd:  4        package_cnt:  2
 *   data: uint8_t[8]        data: uint8_t[8]
 *   ...
 */
#ifndef SPISTREAM_PACKAGE_SIZE
#define SPISTREAM_PACKAGE_SIZE 32
#endif
struct __attribute__((packed)) AutopilotMessagePTStream {
  uint8_t message_cnt;
  int8_t package_cntd;
  uint8_t pkg_data[SPISTREAM_PACKAGE_SIZE];
};

/* Union for computing size of SPI transfer (largest of either up or down message) */
union AutopilotMessage {
  struct OVERO_LINK_MSG_UP msg_up;
  struct OVERO_LINK_MSG_DOWN msg_down;
};

struct __attribute__((packed)) AutopilotMessageCRCFrame {
  union AutopilotMessage payload;
  uint8_t crc;
};

#endif /* FMS_AUTOPILOT_H */
