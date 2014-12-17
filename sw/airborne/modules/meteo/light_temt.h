#ifndef LIGHT_TEMT_H
#define LIGHT_TEMT_H

#include <std.h>

extern uint16_t adc_light_temt;
void light_temt_init(void);
void light_temt_periodic(void);

#endif
