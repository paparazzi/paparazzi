#ifndef AMI601_H
#define AMI601_H

#include "std.h"

extern void ami601_init( void );

extern void ami601_periodic( void );

#define AMI601_NB_CHAN 6
extern uint16_t ami601_val[AMI601_NB_CHAN];
extern uint8_t ami601_foo1;
extern uint8_t ami601_foo2;
extern uint8_t ami601_foo3;

float ami601_ax;
float ami601_ay;
float ami601_az;

float ami601_mx;
float ami601_my;
float ami601_mz;


#endif /* AMI601_H */
