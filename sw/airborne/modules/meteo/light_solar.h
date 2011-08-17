#ifndef TEMP_SOLAR_H
#define TEMP_SOLAR_H

#include "std.h"

#define LIGHT_NB 10

extern uint16_t up[LIGHT_NB], dn[LIGHT_NB];
extern int32_t  light_cnt;

void light_solar_init(void);
void light_solar_periodic(void);

#endif
