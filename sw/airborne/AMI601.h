#ifndef AMI601_H
#define AMI601_H

#include "std.h"

extern void ami601_init( void );

extern void ami601_periodic( void );
extern void ami601_scale_measures(void);

#define AMI601_NB_CHAN 6
extern uint16_t ami601_val[AMI601_NB_CHAN];
extern uint8_t ami601_foo1;
extern uint8_t ami601_foo2;
extern uint8_t ami601_foo3;

extern float ami601_ax;
extern float ami601_ay;
extern float ami601_az;

extern float ami601_mx;
extern float ami601_my;
extern float ami601_mz;


#endif /* AMI601_H */
