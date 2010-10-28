#ifndef TEMP_TEMOD_H
#define TEMP_TEMOD_H

#include "std.h"

#define TEMOD_I2C_R1    256.
#define TEMOD_I2C_R2    128.
#define TEMOD_I2C_R3    64.

extern float ftmd_temperature;

void temod_init(void);
void temod_periodic(void);
void temod_event(void);

#endif
