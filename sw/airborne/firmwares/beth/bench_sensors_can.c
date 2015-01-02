#include "bench_sensors.h"
#include "mcu_periph/can.h"
#include "led.h"

//uint16_t halfw1,halfw2,halfw3,halfw4;
uint16_t can_err_flags = 0;

struct BenchSensors bench_sensors;

static void can_rx_callback(uint32_t id, uint8_t *buf, int len);
static uint8_t rx_bd1 = 0;
static uint8_t rx_bd2 = 0;


void bench_sensors_init(void)
{
  can_init(can_rx_callback);
}

//for now Azimuth board(BD1) is slow so we give it more time...
#define MAX_ALLOWED_SKIPS_CANBD1 (60)
#define MAX_ALLOWED_SKIPS_CANBD2 (10)

void read_bench_sensors(void)
{
  //rx_bd1/2 keep track of how long it's been since last receive (when it is set to 0)
  //if we've lost link for 0.5 second, stop counting (to avoid overflowing our uint8)
  if (rx_bd1 < 255) { rx_bd1++; }
  if (rx_bd2 < 255) { rx_bd2++; }

  //if we haven't heard from a board for 15 periods (15/512=29ms)
  //we flag a CAN error to show which board is at cause and how long it's been.
  can_err_flags = 0;
  if (rx_bd1 > MAX_ALLOWED_SKIPS_CANBD1) {can_err_flags = 1000 + rx_bd1;}
  if (rx_bd2 > MAX_ALLOWED_SKIPS_CANBD2) {can_err_flags += (2000 + rx_bd2);}
  //if ((rx_bd1 > 15) && (rx_bd2 > 15)) {can_err_flags = 3000 +  rx_bd2 + rx_bd1;}
}

static void can_rx_callback(uint32_t id, uint8_t *buf, int len)
{
  //LED_TOGGLE(7);
  static uint16_t tempangle;

  bench_sensors.current = id >> 7;
  if (bench_sensors.current == 2) {
    tempangle = (buf[1] << 8) | buf[0];
    if ((tempangle == 0) || (tempangle > 6000)) {can_err_flags = 0x20;} else {bench_sensors.angle_2 = tempangle;}
    tempangle = (buf[3] << 8) | buf[2];
    if ((tempangle == 0) || (tempangle > 6000)) {can_err_flags = 0x20;} else {bench_sensors.angle_3 = tempangle;}
    rx_bd2 = 0;
    LED_TOGGLE(4);
  } else {
    tempangle = (buf[3] << 8) | buf[2];
    if ((tempangle == 0) || (tempangle > 6000)) {can_err_flags = 0x10;} else {bench_sensors.angle_1 = tempangle;}
    rx_bd1 = 0;
    //LED_TOGGLE(5);
  }
}
