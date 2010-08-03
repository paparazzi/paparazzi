#include "bench_sensors.h"
#include "can.h"
#include "led.h"

uint16_t halfw1,halfw2,halfw3,halfw4;

struct BenchSensors bench_sensors;

static void can_rx_callback(uint32_t id, uint8_t *buf, int len);


void bench_sensors_init(void) {
  can_init(can_rx_callback);
}

void read_bench_sensors(void) {
  
}

static void can_rx_callback(uint32_t id, uint8_t *buf, int len) {
  //LED_TOGGLE(7);
  bench_sensors.current = id>>7;
  if (bench_sensors.current == 2) {
    halfw2 = buf[3];
    halfw2 = (halfw2<<8) + buf[2];
    halfw1 = buf[1];
    halfw1 = (halfw1<<8) + buf[0];
    bench_sensors.angle_2 = halfw1;
    bench_sensors.angle_3 = halfw2;
  } 
  else {
    halfw4 = buf[3];
    halfw4 = (halfw4<<8) + buf[2];
    //halfw3 = buf[1];
    //halfw3 = (halfw3<<8) + buf[0];
    bench_sensors.angle_1 = halfw4;
  }

}
