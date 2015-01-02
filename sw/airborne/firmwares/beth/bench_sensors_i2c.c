#include "bench_sensors.h"

struct BenchSensors bench_sensors, bench_sensors2;


void bench_sensors_init(void)
{
  bench_sensors.status = BS_IDLE;
  bench_sensors.i2c_done = TRUE;
}

void bench_sensors2_init(void)
{
  bench_sensors2.status = BS_IDLE;
  bench_sensors2.i2c_done = TRUE;
}


void read_bench_sensors(void)
{

  const uint8_t bench_addr = 0x40;
  bench_sensors.status = BS_BUSY;
  bench_sensors.i2c_done = FALSE;
  i2c2_receive(bench_addr, 4, &bench_sensors.i2c_done);

}


void read_bench_sensors2(void)
{

  const uint8_t bench_addr2 = 0x30;
  bench_sensors2.status = BS_BUSY;
  bench_sensors2.i2c_done = FALSE;
  i2c2_receive(bench_addr2, 4, &bench_sensors2.i2c_done);

}
