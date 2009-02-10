#include "booz2_battery.h"

uint8_t booz2_battery_voltage;

extern void booz2_battery_init(void) {
  booz2_battery_voltage = 0;
}
