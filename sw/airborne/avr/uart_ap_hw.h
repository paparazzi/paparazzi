#include <avr/io.h>
#include <avr/signal.h>
#include <avr/interrupt.h>

#define ReceiveUart0(cb) \
  SIGNAL( SIG_UART0_RECV ) { \
    uint8_t c = UDR0; \
    cb(c); \
}
#define ReceiveUart1(cb) \
  SIGNAL( SIG_UART1_RECV ) { \
    uint8_t c = UDR1; \
    cb(c); \
}
