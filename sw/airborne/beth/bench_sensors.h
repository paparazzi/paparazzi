#ifndef BENCH_SENSORS_H
#define BENCH_SENSORS_H

#include "std.h"

#ifdef USE_I2C2
#include "i2c.h"
#endif

#ifdef USE_CAN1
#include "can.h"
#include "can_hw.h"
#include <stm32/can.h>

extern volatile uint8_t CAN_RX_FLAG;
extern CanRxMsg can_rx_msg;
extern uint16_t halfw1,halfw2,halfw3,halfw4,tempid;
#endif

extern void bench_sensors_init(void);
extern void read_bench_sensors(void);

enum BenchSensorsStatus { BS_IDLE, BS_BUSY, BS_AVAILABLE};

struct BenchSensors {
  enum BenchSensorsStatus status;
  uint16_t angle_1;
  uint16_t angle_2;
  uint16_t angle_3;
  uint16_t current;
  bool_t   ready;
};


extern struct BenchSensors bench_sensors;

#ifdef USE_I2C2
#define BenchSensorsEvent( _handler) {		\
    if (bench_sensors.status ==  BS_BUSY && bench_sensors.ready) {	\
      bench_sensors.angle_1 = i2c2.buf[0] + (i2c2.buf[1] << 8);		\
      bench_sensors.angle_2 = i2c2.buf[2] + (i2c2.buf[3] << 8);		\
      bench_sensors.status = BS_IDLE;					\
      _handler();							\
    }									\
  }
#endif


#ifdef USE_CAN1
#define BenchSensorsEvent( _handler) {		\
    if (CAN_RX_FLAG == 1) {	\
	tempid = (uint16_t)(can_rx_msg.ExtId>>7); \
	if (tempid == 2) { \
		bench_sensors.current = 2;\
		halfw2 = can_rx_msg.Data[3]; \
		halfw2 = (halfw2<<8) + can_rx_msg.Data[2]; \
		halfw1 = can_rx_msg.Data[1]; \
		halfw1 = (halfw1<<8) + can_rx_msg.Data[0]; \
		bench_sensors.angle_2 = halfw1;		\
		bench_sensors.angle_3 = halfw2;		\
	} else { \
		bench_sensors.current = 1;\
		halfw4 = can_rx_msg.Data[3]; \
		halfw4 = (halfw4<<8) + can_rx_msg.Data[2]; \
		halfw3 = can_rx_msg.Data[1]; \
		halfw3 = (halfw3<<8) + can_rx_msg.Data[0]; \
		bench_sensors.angle_1 = halfw4;		\
	} \
      CAN_RX_FLAG = 0;					\
      _handler();							\
    }									\
  }
#endif

#endif /* BENCH_SENSORS_H  */
