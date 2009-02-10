#ifndef BOOZ2_BATTERY_H
#define BOOZ2_BATTERY_H

#include "std.h"

#include "airframe.h"

/* decivolts */
extern uint8_t booz2_battery_voltage;

#define Booz2BatteryISRHandler(_val) {					\
    uint32_t cal_v = (uint32_t)(_val) * BATTERY_SENS_NUM / BATTERY_SENS_DEN; \
    uint32_t sum = (uint32_t)booz2_battery_voltage + cal_v;		\
    booz2_battery_voltage = (uint8_t)(sum/2);				\
  }


extern void booz2_battery_init(void);

#endif /* BOOZ2_BATTERY_H */
