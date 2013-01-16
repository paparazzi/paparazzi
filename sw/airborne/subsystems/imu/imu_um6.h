/*
 * Released under Creative Commons License
 *
 * 2012, Utah State University, http://aggieair.usu.edu/
 */
#ifndef IMU_UM6_H
#define IMU_UM6_H

#include "generated/airframe.h"
#include "subsystems/imu.h"
#include "mcu_periph/uart.h"
#include "subsystems/ahrs.h"
#include "subsystems/ins.h"

#define __UM6Link(dev, _x) dev##_x
#define _UM6Link(dev, _x)  __UM6Link(dev, _x)
#define UM6Link(_x) _UM6Link(UM6_LINK, _x)

#define UM6Buffer() UM6Link(ChAvailable())

// GYRO
#if !defined IMU_GYRO_P_SENS & !defined IMU_GYRO_Q_SENS & !defined IMU_GYRO_R_SENS
#define IMU_GYRO_P_SENS 1
#define IMU_GYRO_P_SENS_NUM 1
#define IMU_GYRO_P_SENS_DEN 1
#define IMU_GYRO_Q_SENS 1
#define IMU_GYRO_Q_SENS_NUM 1
#define IMU_GYRO_Q_SENS_DEN 1
#define IMU_GYRO_R_SENS 1
#define IMU_GYRO_R_SENS_NUM 1
#define IMU_GYRO_R_SENS_DEN 1
#endif
#if !defined IMU_GYRO_P_SIGN & !defined IMU_GYRO_Q_SIGN & !defined IMU_GYRO_R_SIGN
#define IMU_GYRO_P_SIGN   1
#define IMU_GYRO_Q_SIGN   1
#define IMU_GYRO_R_SIGN   1
#endif
#if !defined IMU_GYRO_P_NEUTRAL & !defined IMU_GYRO_Q_NEUTRAL & !defined IMU_GYRO_R_NEUTRAL
#define IMU_GYRO_P_NEUTRAL 0
#define IMU_GYRO_Q_NEUTRAL 0
#define IMU_GYRO_R_NEUTRAL 0
#endif

// ACCELEROMETER
#if !defined IMU_ACCEL_X_SENS & !defined IMU_ACCEL_Y_SENS & !defined IMU_ACCEL_Z_SENS
#define IMU_ACCEL_X_SENS 1
#define IMU_ACCEL_X_SENS_NUM 1
#define IMU_ACCEL_X_SENS_DEN 1
#define IMU_ACCEL_Y_SENS 1
#define IMU_ACCEL_Y_SENS_NUM 1
#define IMU_ACCEL_Y_SENS_DEN 1
#define IMU_ACCEL_Z_SENS 1
#define IMU_ACCEL_Z_SENS_NUM 1
#define IMU_ACCEL_Z_SENS_DEN 1
#endif
#if !defined IMU_ACCEL_X_SIGN & !defined IMU_ACCEL_Y_SIGN & !defined IMU_ACCEL_Z_SIGN
#define IMU_ACCEL_X_SIGN  1
#define IMU_ACCEL_Y_SIGN  1
#define IMU_ACCEL_Z_SIGN  1
#endif
#if !defined IMU_ACCEL_X_NEUTRAL & !defined IMU_ACCEL_Y_NEUTRAL & !defined IMU_ACCEL_Z_NEUTRAL
#define IMU_ACCEL_X_NEUTRAL 0
#define IMU_ACCEL_Y_NEUTRAL 0
#define IMU_ACCEL_Z_NEUTRAL 0
#endif

// MAGNETOMETER
#if !defined IMU_MAG_X_SENS & !defined IMU_MAX_Y_SENS & !defined IMU_MAG_Z_SENS
#define IMU_MAG_X_SENS 1
#define IMU_MAG_X_SENS_NUM 1
#define IMU_MAG_X_SENS_DEN 1
#define IMU_MAX_Y_SENS 1
#define IMU_MAG_Y_SENS_NUM 1
#define IMU_MAG_Y_SENS_DEN 1
#define IMU_MAG_Z_SENS 1
#define IMU_MAG_Z_SENS_NUM 1
#define IMU_MAG_Z_SENS_DEN 1
#endif
#if !defined IMU_MAG_X_SIGN & !defined IMU_MAG_Y_SIGN & !defined IMU_MAG_Z_SIGN
#define IMU_MAG_X_SIGN 1
#define IMU_MAG_Y_SIGN 1
#define IMU_MAG_Z_SIGN 1
#endif
#if !defined IMU_MAG_X_NEUTRAL & !defined IMU_MAG_Y_NEUTRAL & !defined IMU_MAG_Z_NEUTRAL
#define IMU_MAG_X_NEUTRAL 0
#define IMU_MAG_Y_NEUTRAL 0
#define IMU_MAG_Z_NEUTRAL 0
#endif

#define IMU_UM6_BUFFER_LENGTH 32
#define IMU_UM6_DATA_OFFSET 5
#define IMU_UM6_LONG_DELAY 4000000

#define IMU_UM6_COMMUNICATION_REG 0x00
#define IMU_UM6_MISC_CONFIG_REG 0x01
#define IMU_UM6_GET_FIRMWARE_CMD 0xAA
#define IMU_UM6_ZERO_GYROS_CMD 0xAC
#define IMU_UM6_RESET_EKF_CMD 0xAD  
#define IMU_UM6_GET_DATA 0xAE
#define IMU_UM6_SET_ACCEL_REF 0xAF
#define IMU_UM6_SET_MAG_REF 0xB0  

#define IMU_UM6_GYRO_PROC 0x5C
#define IMU_UM6_ACCEL_PROC 0x5E
#define IMU_UM6_MAG_PROC 0x60
#define IMU_UM6_EULER 0x62
#define IMU_UM6_QUAT 0x64

extern void UM6_packet_read_message(void);
extern void UM6_packet_parse(uint8_t c);
extern void UM6_packet_send(uint8_t *data);
inline uint16_t UM6_calculate_checksum(uint8_t packet_buffer[], uint8_t packet_length);

extern struct UM6Packet UM6_packet;

extern uint8_t PacketLength;
extern uint8_t PacketType;
extern uint8_t PacketAddr;

extern uint16_t chk_calc;
extern uint16_t chk_rec;

extern enum UM6Status UM6_status;
extern volatile uint8_t UM6_imu_available;

extern struct FloatEulers UM6_eulers;
extern struct FloatQuat UM6_quat;

struct UM6Packet {
  bool_t  msg_available;
  uint32_t chksm_error;
  uint32_t hdr_error;
  uint8_t msg_buf[IMU_UM6_BUFFER_LENGTH];
  uint8_t  status;
  uint8_t  msg_idx;
};

enum UM6PacketStatus {
    UM6PacketWaiting,
    UM6PacketReadingS,
    UM6PacketReadingN,
    UM6PacketReadingPT,
    UM6PacketReadingAddr,
    UM6PacketReadingData
};

enum UM6Status {
  UM6Uninit,
  UM6Running
};

static inline bool_t UM6_verify_chk(uint8_t packet_buffer[], uint8_t packet_length) {
    chk_rec = (packet_buffer[packet_length-2] << 8) | packet_buffer[packet_length-1];
    chk_calc = UM6_calculate_checksum(packet_buffer, packet_length-2);    
    return (chk_calc == chk_rec);
}

static inline void UM6_send_packet(uint8_t *packet_buffer, uint8_t packet_length) {
  for (int i=0; i<packet_length; i++) {
    UM6Link(Transmit(packet_buffer[i]));  
  }
}

inline uint16_t UM6_calculate_checksum(uint8_t packet_buffer[], uint8_t packet_length) {
  uint16_t chk = 0;
  for (int i=0; i<packet_length; i++) {
    chk += packet_buffer[i];
  }
  return chk;
}
		
#define imu_um6_event(_callback1, _callback2, _callback3) {				\
    if (UM6Buffer()) {							\
      ReadUM6Buffer();							\
    }									\
    if (UM6_packet.msg_available) {					\
      UM6_packet.msg_available = FALSE;					\
      UM6_packet_read_message();					\
      _callback1();			\
      _callback2();			\
      _callback3();     \
    } \
}

#define ReadUM6Buffer() {						\
    while (UM6Link(ChAvailable())&&!UM6_packet.msg_available)		\
      UM6_packet_parse(UM6Link(Getch()));				\
  }

#define ImuEvent(_gyro_handler, _accel_handler, _mag_handler) { \
  imu_um6_event(_gyro_handler, _accel_handler, _mag_handler); \
}

#endif /* IMU_UM6_H*/
