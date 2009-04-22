#ifndef CSC_SERVOS_H
#define CSC_SERVOS_H

#include <inttypes.h>

/* 
   lpc2129 pwm pinout

LPC   shared         port   csc servo
PWM1  TXD0           P0.0    4   
PWM2  SSEL0  EINT2   P0.7    0
PWM3  RXD0   EINT0   P0.1    5 
PWM4  TXD1   AD1_1   P0.8    1
PWM5  AD1_6  CAP1_3  P0.21   2
PWM6  RXD1   EINT3   P0.9    3

*/

extern void csc_servos_init(void);
extern void csc_servos_set(int32_t* val);



#endif /* CSC_SERVOS_H */

