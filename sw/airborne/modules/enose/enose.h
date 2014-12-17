#ifndef ENOSE_H
#define ENOSE_H

#include "std.h"

#ifdef ENOSE
#if !defined USE_I2C && !defined SITL
#define USE_I2C
#endif
#endif


#define ENOSE_NB_SENSOR 3

extern uint8_t enose_heat[ENOSE_NB_SENSOR];
extern uint16_t enose_val[ENOSE_NB_SENSOR];
extern uint16_t enose_PID_val;

#define ENOSE_IDLE         0
#define ENOSE_SETTINGS     1
#define ENOSE_MEASURING_WR 2
#define ENOSE_MEASURING_RD 3


extern uint8_t enose_status;

extern void enose_init(void);

extern void enose_set_heat(uint8_t no_sensor, uint8_t value);
extern void enose_periodic(void);

#define enose_SetHeat0(val) {enose_set_heat(0, val);}
#define enose_SetHeat1(val) {enose_set_heat(1, val);}
#define enose_SetHeat2(val) {enose_set_heat(2, val);}

#define enose_DecreaseVal0(_x) { enose_val[0] -= _x; }
#define enose_DecreaseVal1(_x) { enose_val[1] -= _x; }
#define enose_DecreaseVal2(_x) { enose_val[2] -= _x; }


#endif /* ENOSE_H */
