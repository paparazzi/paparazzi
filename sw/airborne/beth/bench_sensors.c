#include "bench_sensors.h"

#include "i2c.h"

bool_t   bench_sensors_available;
uint16_t bench_sensors_angle_1;
uint16_t bench_sensors_angle_2;
uint16_t bench_sensors_angle_3;
uint16_t bench_sensors_current;

void bench_sensors_init(void) {
  bench_sensors_available = FALSE;
}


void read_bench_sensors(void) {

  const uint8_t bench_addr = 0x52;

  i2c1_receive(bench_addr, 4, &bench_sensors_available);
  
}
