#ifndef _UART_H_
#define _UART_H_

#include <inttypes.h>
#include <avr/io.h>
#include <avr/signal.h>
#include <avr/interrupt.h>



/*************************************************************************
 *
 *  UART code.
 */

void uart_init( void );
void uart_putc( unsigned char c );
extern uint8_t uart_nb_ovrrun;
#endif
