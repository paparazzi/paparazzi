#ifndef SOFT_UART_H
#define SOFT_UART_H

#include <inttypes.h>

extern volatile uint8_t soft_uart_got_byte;
extern uint8_t soft_uart_byte;

#define RX_ERROR_FRAMING 1
#define RX_ERROR_OVERRUN 2       
extern volatile uint8_t soft_uart_error;

#define SOFT_UART_CD_PORT PORTD
#define SOFT_UART_CD_DDR  DDRD
#define SOFT_UART_CD_PIN  PIND
#define SOFT_UART_CD      6

void soft_uart_init(void);


#endif
