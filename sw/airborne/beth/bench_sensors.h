#ifndef BENCH_SENSORS_H
#define BENCH_SENSORS_H

#include "std.h"

extern void bench_sensors_init(void);
extern void read_bench_sensors(void);
extern bool_t   bench_sensors_available;
extern uint16_t bench_sensors_angle_1;
extern uint16_t bench_sensors_angle_2;
extern uint16_t bench_sensors_angle_3;
extern uint16_t bench_sensors_current;

#define BenchSensorsEvent( _handler) {		\
    if (bench_sensors_available) {		\
      bench_sensors_angle_1 = i2c1_buf[0];	\
      bench_sensors_angle_2 = i2c1_buf[1];	\
      bench_sensors_angle_3 = i2c1_buf[2];	\
      bench_sensors_current = i2c1_buf[3];	\
      _handler();				\
      bench_sensors_available = FALSE;		\
    }						\
  }



#endif /* BENCH_SENSORS_H  */
