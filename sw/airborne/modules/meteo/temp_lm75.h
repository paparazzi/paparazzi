#ifndef TEMP_LM75_H
#define TEMP_LM75_H

#include "std.h"

#define LM75_TEMP_REG       0x00
#define LM75_CONF_REG       0x01
#define LM75_T_HYST_REG     0x02
#define LM75_T_OS_REG       0x03
#define LM75_PROD_REG       0x07


void lm75_init(void);
void lm75_periodic(void);
void lm75_event(void);

#endif
