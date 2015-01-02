#ifndef BENCH_SENSORS_H
#define BENCH_SENSORS_H

#include "std.h"

#if USE_I2C2
#include "i2c.h"
#endif

#ifdef USE_CAN1
#include "mcu_periph/can.h"
extern uint16_t can_err_flags;
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

#if USE_I2C2
#define BenchSensorsEvent( _handler) {    \
    if (bench_sensors.status ==  BS_BUSY && bench_sensors.ready) {  \
      bench_sensors.angle_1 = i2c2.buf[0] + (i2c2.buf[1] << 8);   \
      bench_sensors.angle_2 = i2c2.buf[2] + (i2c2.buf[3] << 8);   \
      bench_sensors.status = BS_IDLE;         \
      _handler();             \
    }                 \
  }
#endif

#endif /* BENCH_SENSORS_H  */
