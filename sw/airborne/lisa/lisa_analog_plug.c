#include "std.h"

uint16_t booz2_analog_baro_offset;
uint16_t booz2_analog_baro_value;
uint16_t booz2_analog_baro_value_filtered;
bool_t   booz2_analog_baro_data_available;
uint16_t booz2_analog_baro_status;

uint8_t booz2_battery_voltage;

void booz2_analog_init(void);
void booz2_analog_baro_init(void);
void booz2_battery_init(void);

void booz2_analog_init(void) {}

void booz2_analog_baro_init(void) {}

void booz2_battery_init(void) {}

