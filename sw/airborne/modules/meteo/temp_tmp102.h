#ifndef TEMP_TMP102_H
#define TEMP_TMP102_H

#include "std.h"

#define TMP102_TEMP_REG     0x00
#define TMP102_CONF_REG     0x01
#define TMP102_T_LOW_REG    0x02
#define TMP102_T_HIGH_REG   0x03

extern float ftmp_temperature;

void tmp102_init(void);
void tmp102_periodic(void);
void tmp102_event(void);

#endif
