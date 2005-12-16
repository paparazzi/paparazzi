#ifndef MODEM_H
#define MODEM_H

#include "types.h"

void modem_init ( void );
void modem_put_one_byte( uint8_t _byte);

void TIMER1_ISR ( void ) __attribute__((naked));


#endif /* MODEM_H */
