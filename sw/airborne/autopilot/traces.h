#ifndef TRACES_H
#define TRACES_H

#include "uart.h"

extern void uart0_print_string(const uint8_t*);
extern void uart0_print_hex(const uint8_t);
extern void uart0_print_hex16(const uint16_t);


#endif /* TRACES_H */
