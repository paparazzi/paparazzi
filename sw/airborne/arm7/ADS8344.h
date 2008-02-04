#ifndef ADS8344_H
#define ADS8344_H

#include "std.h"

#define NB_CHANNELS 8

extern uint16_t ADS8344_values[NB_CHANNELS];

void ADS8344_init( void );
void ADS8344_start( void );

#endif // ADS8344_H
