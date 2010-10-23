#ifndef DUST_GP2Y_H
#define DUST_GP2Y_H

#include "std.h"

#define DUST_GP2Y_UNINIT     0
#define DUST_GP2Y_IDLE       1

extern uint8_t  dust_gp2y_status;
extern uint16_t dust_gp2y_density;
extern float dust_gp2y_density_f;

void dust_gp2y_init(void);
void dust_gp2y_periodic(void);
void dust_gp2y_event(void);

#endif

