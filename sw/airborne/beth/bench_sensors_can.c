#include "bench_sensors.h"
#include "can.h"

uint16_t halfw1,halfw2,halfw3,halfw4,tempid;

struct BenchSensors bench_sensors;

void bench_sensors_init(void) {
  can_init();
}

void read_bench_sensors(void) {
  
}

