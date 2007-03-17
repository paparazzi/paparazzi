#ifndef ENOSE_H
#define ENOSE_H

#include "std.h"

#define ENOSE_NB_SENSOR 3

extern uint8_t enose_heat[ENOSE_NB_SENSOR];
extern uint16_t enose_val[ENOSE_NB_SENSOR];

#define ENOSE_IDLE         0
#define ENOSE_SETTINGS     1
#define ENOSE_MEASURING_WR 2
#define ENOSE_MEASURING_RD 3


extern uint8_t enose_status;

extern void enose_init( void );

extern void enose_set_heat(uint8_t no_sensor, uint8_t value);
extern void enose_periodic( void );

#define enose_SetHeat0(val) {enose_set_heat(0, val);}
#define enose_SetHeat1(val) {enose_set_heat(1, val);}
#define enose_SetHeat2(val) {enose_set_heat(2, val);}


#endif /* ENOSE_H */
