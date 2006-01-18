#ifndef TRACES_H
#define TRACES_H

#include "uart_ap_hw.h"

//#define _TRACES_INIT(s) s ## _init()
//#define TRACES_INIT() _TRACES_INIT(TRACES)

#define TRACES_INIT() uart1_init()

#define PRINT_RADIO_CONTROL() {			\
  uart1Puts("RC ");				\
  uart1_print_hex_16(rc_values[0]);		\
  uart1Puts(", ");				\
  uart1_print_hex_16(rc_values[1]);		\
  uart1Puts(", ");				\
  uart1_print_hex_16(rc_values[2]);		\
  uart1Puts(", ");				\
  uart1_print_hex_16(rc_values[3]);		\
  uart1Puts(", ");				\
  uart1_print_hex_16(rc_values[4]);		\
  uart1Puts(", ");				\
  uart1_print_hex_16(rc_values[5]);		\
  uart1Puts(", ");				\
  uart1_print_hex_16(rc_values[6]);		\
  uart1Puts("\r\n");				\
}


#define PRINT_CONTROL_COMMANDS() {		\
    uart1Puts("CM ");				\
    uart1_print_hex_16(control_commands[0]);	\
    uart1Puts(", ");				\
    uart1_print_hex_16(control_commands[1]);	\
    uart1Puts(", ");				\
    uart1_print_hex_16(control_commands[2]);	\
    uart1Puts("\r\n");				\
}

extern void uart1_print_hex( uint8_t c);
extern void uart1_print_hex_16( uint16_t c);



#endif /* TRACES_H */
